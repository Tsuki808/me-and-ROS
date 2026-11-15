#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Windows operation desk for Qwen3-VL and ROS1 semantic pick."""
import argparse
import base64
import json
import os
import threading
import time
import uuid
import pandas as pd
from urllib import request as urlrequest

import cv2
import numpy as np
import gradio as gr
import roslibpy
from roslibpy import ServiceRequest

SCHEMA = {
    'type': 'object', 'additionalProperties': False,
    'properties': {
        'decision': {'type': 'string', 'enum': ['answer', 'pick_show_return', 'clarify']},
        'candidate_id': {'type': 'string'}, 'summary': {'type': 'string'}},
    'required': ['decision', 'candidate_id', 'summary']}
FIXED_PROMPT = '拾取当前视野中 YOLO 检测置信度最高的目标盒子，完成举示动作后放回原位'

class Desk(object):
    def __init__(self, args):
        self.args, self.latest, self.lock, self.pending = args, None, threading.Lock(), None
        self.last_model_seconds = None
        self.status_lock = threading.Lock()
        self.current_status = '等待 ROS 状态'
        self.ros = roslibpy.Ros(host=args.ros_host, port=args.ros_port)
        self.ros.run()
        self.observation = roslibpy.Topic(self.ros, '/final_project/embodied/observation', 'std_msgs/String')
        self.observation.subscribe(self.on_observation)
        self.status_topic = roslibpy.Topic(self.ros, '/final_project/status', 'std_msgs/String')
        self.status_topic.subscribe(self.on_status)
        self.heartbeat = roslibpy.Topic(self.ros, '/final_project/embodied/heartbeat', 'std_msgs/String')
        self.heartbeat.advertise()
        thread = threading.Thread(target=self.publish_heartbeat)
        thread.daemon = True
        thread.start()
        action_root = '/final_project/embodied/pick_show_return'
        self.action_goal = roslibpy.Topic(
            self.ros, action_root + '/goal',
            'final_project/SemanticPickActionGoal')
        self.action_result = roslibpy.Topic(
            self.ros, action_root + '/result',
            'final_project/SemanticPickActionResult')
        self.action_result.subscribe(self.on_action_result)
        self.action_lock = threading.Lock()
        self.action_waiters = {}
        self.cancel = roslibpy.Service(self.ros, '/final_project/embodied/cancel', 'std_srvs/Trigger')

    def on_observation(self, message):
        try:
            value = json.loads(message['data'])
            if value.get('observation_id'):
                with self.lock:
                    self.latest = value
        except (KeyError, TypeError, ValueError):
            pass

    def on_status(self, message):
        value = message.get('data', '')
        with self.status_lock:
            self.current_status = value

    def on_action_result(self, message):
        status = message.get('status', {})
        goal_id = status.get('goal_id', {}).get('id')
        if not goal_id:
            return
        with self.action_lock:
            waiter = self.action_waiters.get(goal_id)
            if waiter is not None:
                waiter['result'] = message.get('result', {})
                waiter['event'].set()

    def publish_heartbeat(self):
        while self.ros.is_connected:
            try:
                self.heartbeat.publish(roslibpy.Message({'data': 'windows-desk'}))
            except Exception:
                return
            time.sleep(1.)

    def snapshot(self):
        with self.lock:
            return json.loads(json.dumps(self.latest)) if self.latest else None

    def status_snapshot(self):
        with self.status_lock:
            return self.current_status

    @staticmethod
    def image_value(encoded):
        if not encoded:
            return None
        raw = base64.b64decode(encoded.encode('ascii'))
        image = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB) if image is not None else None

    def ask_model(self, instruction, reference):
        observation = self.snapshot()
        if not observation or not observation.get('image_jpeg'):
            return None, '尚未收到带图像的 ROS 观察帧。'
        prompt = ('任务指令：%s\n'
                  '根据当前相机画面和候选列表选择目标。'
                  '如有第二张图，它是用户提供的参考图。只可选择候选列表中的 candidate_id；'
                  '目标明确且存在时返回 pick_show_return，不明确或不存在时返回 clarify，'
                  '非取物请求返回 answer。不要生成坐标、速度或关节角。\n'
                  '候选：%s' %
                  (FIXED_PROMPT,
                   json.dumps(observation.get('candidates', []), ensure_ascii=False)))
        content = [{'type': 'text', 'text': prompt},
                   {'type': 'image_url', 'image_url': {'url': 'data:image/jpeg;base64,' + observation['image_jpeg']}}]
        if reference and os.path.isfile(reference):
            with open(reference, 'rb') as stream:
                content.append({'type': 'image_url', 'image_url': {
                    'url': 'data:image/jpeg;base64,' + base64.b64encode(stream.read()).decode('ascii')}})
        payload = {'model': self.args.model, 'messages': [{'role': 'user', 'content': content}],
                   'temperature': 0, 'max_tokens': 256,
                   'response_format': {'type': 'json_schema',
                                       'json_schema': {'name': 'semantic_decision',
                                                       'schema': SCHEMA}}}
        request = urlrequest.Request(self.args.llama_url + '/v1/chat/completions',
                                     data=json.dumps(payload).encode('utf-8'),
                                     headers={'Content-Type': 'application/json'})
        started = time.time()
        try:
            with urlrequest.urlopen(request, timeout=90) as response:
                answer = json.loads(response.read().decode('utf-8'))
            text = answer['choices'][0]['message']['content']
            decision = json.loads(text) if isinstance(text, str) else text
            self.pending = (observation, decision)
            self.last_model_seconds = time.time() - started
            return observation, decision.get('summary', '')
        except Exception as exc:
            self.last_model_seconds = time.time() - started
            return None, '模型请求失败：%s' % exc

    @staticmethod
    def highest_candidate(observation):
        candidates = (observation or {}).get('candidates', [])
        if not candidates:
            return None
        return max(candidates, key=lambda item: float(item.get('confidence', 0.)))

    @staticmethod
    def candidate_rows(observation):
        rows = []
        for item in (observation or {}).get('candidates', []):
            rows.append([item.get('candidate_id', ''), item.get('color', 'unknown'),
                         item.get('label', ''), round(float(item.get('confidence', 0.)), 4),
                         str(item.get('bbox', ''))])
        return rows

    @staticmethod
    def confidence_data(observation):
        rows = []
        for item in (observation or {}).get('candidates', []):
            rows.append({'candidate': '%s · %s' % (item.get('candidate_id', ''),
                          item.get('color', 'unknown')),
                          'confidence': float(item.get('confidence', 0.)),
                          'color': item.get('color', 'unknown')})
        return pd.DataFrame(rows, columns=['candidate', 'confidence', 'color'])

    def decision_record(self, observation, decision, requested_color=None, execution=None):
        record = dict(decision)
        record['observation_id'] = observation.get('observation_id')
        record['requested_color'] = requested_color
        record['qwen_latency_seconds'] = round(self.last_model_seconds or 0., 3)
        selected = next((item for item in observation.get('candidates', [])
                         if item.get('candidate_id') == decision.get('candidate_id')), None)
        record['selected_candidate'] = selected
        if execution is not None:
            record['execution'] = execution
        return record

    @staticmethod
    def metrics_text(observation, decision, model_seconds=None, phase='等待观察'):
        if not observation:
            return '阶段：%s' % phase
        count = len(observation.get('candidates', []))
        confidence = max([float(x.get('confidence', 0.))
                          for x in observation.get('candidates', [])] or [0.])
        elapsed = '未知' if model_seconds is None else '%.2f s' % model_seconds
        return ('观测 `%s` · 候选 %d 个 · 最高 YOLO 置信度 %.3f · '
                'Qwen 请求 %s · 阶段：%s' %
                (observation.get('observation_id', ''), count, confidence,
                 elapsed, phase))

    def analyze(self, instruction, reference):
        observation, message = self.ask_model(instruction, reference)
        if not observation:
            return (message, None, '未执行', {}, [], self.confidence_data(None),
                    self.metrics_text(None, None, self.last_model_seconds))
        decision = self.pending[1]
        highest = self.highest_candidate(observation)
        if decision.get('decision') == 'pick_show_return' and highest and decision.get('candidate_id') != highest.get('candidate_id'):
            decision = dict(decision)
            decision['candidate_id'] = highest.get('candidate_id')
            decision['summary'] = ('已按 YOLO 最高置信度候选校正为 %s。%s' %
                                   (highest.get('candidate_id'), decision.get('summary', '')))
            self.pending = (observation, decision)
        phase = ('等待确认执行' if decision.get('decision') == 'pick_show_return'
                 else '已拒绝执行')
        return ('%s\n决策：%s' % (message, decision.get('decision')),
                self.image_value(observation.get('image_jpeg')), phase,
                self.decision_record(observation, decision),
                self.candidate_rows(observation),
                self.confidence_data(observation),
                self.metrics_text(observation, decision, self.last_model_seconds, phase))

    def execute(self):
        if not self.pending:
            yield ('请先分析当前场景。', None, '未执行', {}, [], self.confidence_data(None),
                   self.metrics_text(None, None))
            return
        observation, decision = self.pending
        record = self.decision_record(observation, decision)
        image = self.image_value(observation.get('image_jpeg'))
        if decision.get('decision') != 'pick_show_return':
            yield (decision.get('summary', '目标不明确，已拒绝执行。'), image, '已拒绝执行',
                   record, self.candidate_rows(observation), self.confidence_data(observation),
                   self.metrics_text(observation, decision, self.last_model_seconds, '已拒绝执行'))
            return
        candidate = next((x for x in observation.get('candidates', [])
                          if x.get('candidate_id') == decision.get('candidate_id')), None)
        if not candidate:
            yield ('模型返回的候选已不在当前快照中，已拒绝。', image, '已拒绝执行',
                   record, self.candidate_rows(observation), self.confidence_data(observation),
                   self.metrics_text(observation, decision, self.last_model_seconds, '已拒绝执行'))
            return
        highest = self.highest_candidate(observation)
        if not highest or candidate.get('candidate_id') != highest.get('candidate_id'):
            phase = '候选已非最高置信度，已阻止执行'
            record['execution'] = {'success': False,
                                   'message': '仅允许执行当前 YOLO 置信度最高的候选；未发送 ROS 动作'}
            self.pending = None
            yield ('当前候选已不是 YOLO 置信度最高目标，未发送 ROS 动作。', image, phase,
                   record, self.candidate_rows(observation), self.confidence_data(observation),
                   self.metrics_text(observation, decision, self.last_model_seconds, phase))
            return
        goal_id = 'windows-desk-' + uuid.uuid4().hex
        stamp_seconds = float(observation.get('stamp', 0.))
        stamp = {'secs': int(stamp_seconds),
                 'nsecs': int((stamp_seconds % 1.) * 1000000000)}
        result_box = {'event': threading.Event()}
        with self.action_lock:
            self.action_waiters[goal_id] = result_box
        self.action_goal.publish(roslibpy.Message({
            'header': {'seq': 0, 'stamp': stamp, 'frame_id': ''},
            'goal_id': {'stamp': stamp, 'id': goal_id},
            'goal': {'color': candidate.get('color', ''),
                     'observation_id': observation['observation_id'],
                     'candidate_id': candidate['candidate_id']}}))

        def wait_for_result():
            if not result_box['event'].wait(1800):
                result_box['error'] = 'ROS Action result timed out after 1800 seconds'

        waiter = threading.Thread(target=wait_for_result)
        waiter.daemon = True
        waiter.start()
        yield ('已提交真实 ROS 抓取技能，等待任务阶段反馈。', image, '执行中',
               record, self.candidate_rows(observation), self.confidence_data(observation),
               self.metrics_text(observation, decision, self.last_model_seconds,
                                 '执行中：' + self.status_snapshot()))
        last_status = None
        while waiter.is_alive():
            current = self.status_snapshot()
            if current and current != last_status:
                yield ('ROS 技能执行中：%s' % current, image, '执行中', record,
                       self.candidate_rows(observation), self.confidence_data(observation),
                       self.metrics_text(observation, decision, self.last_model_seconds,
                                         '执行中：' + current))
                last_status = current
            time.sleep(.4)
        waiter.join()
        with self.action_lock:
            self.action_waiters.pop(goal_id, None)
        self.pending = None
        result = result_box.get('result', {})
        success = bool(result.get('success', False))
        phase = success and '完成' or '失败'
        message = result.get('message', result_box.get('error', '动作服务器未返回结果'))
        record = self.decision_record(observation, decision,
                                      execution={'success': success, 'message': message,
                                       'elapsed_seconds': result.get('elapsed_seconds')})
        yield ('%s\n%s' % (decision.get('summary', ''), message), image, phase, record,
                self.candidate_rows(observation),
                self.confidence_data(observation),
                self.metrics_text(observation, decision, self.last_model_seconds, phase))

    def cancel_task(self):
        try:
            response = self.cancel.call(ServiceRequest({}), timeout=3.)
            self.pending = None
            return response.get('message', '已请求取消'), '已取消'
        except Exception as exc:
            return '取消失败：%s' % exc, '取消失败'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ros-host', default='192.168.162.131')
    parser.add_argument('--ros-port', type=int, default=9090)
    parser.add_argument('--llama-url', default='http://127.0.0.1:8080')
    parser.add_argument('--model', default='Qwen3VL-4B-Instruct-Q4_K_M.gguf')
    parser.add_argument('--server-port', type=int, default=7860)
    args = parser.parse_args()
    with gr.Blocks(title='模式2.2 具身语义操作台') as demo:
        gr.Markdown('# 模式 2.2 · 语义取物控制台\n实时相机与 YOLO 候选 → Qwen3-VL 决策 → ROS 抓取、举示、放回')
        with gr.Row():
            instruction = gr.Textbox(label='中文指令', value=FIXED_PROMPT,
                                     interactive=False)
            reference = gr.Image(label='可选参考图', type='filepath')
        with gr.Row():
            analyze = gr.Button('分析场景')
            execute = gr.Button('确认并执行', variant='primary')
            cancel = gr.Button('取消并停车')
        output = gr.Textbox(label='模型与动作反馈', lines=4)
        state = gr.Textbox(label='任务阶段', value='等待观察')
        with gr.Row():
            image = gr.Image(label='实时 ROS 观察帧（YOLO 标注）', type='numpy', scale=7)
            with gr.Column(scale=5):
                confidence_plot = gr.BarPlot(
                    x='candidate', y='confidence', color='color',
                    color_map={'red': '#d94b4b', 'green': '#3a9b68',
                               'blue': '#3979c6', 'yellow': '#d5a62b',
                               'unknown': '#687582'},
                    title='YOLO 候选置信度', x_title='候选目标',
                    y_title='置信度', y_lim=[0, 1], sort='-y', height=300,
                    show_export_button=True)
                decision_view = gr.JSON(label='Qwen 结构化决策', open=True)
        candidates_view = gr.Dataframe(
            headers=['候选编号', '颜色', '标签', '置信度', '边界框'],
            datatype=['str', 'str', 'str', 'number', 'str'],
            label='检测候选明细', interactive=False, show_row_numbers=True)
        metrics_view = gr.Markdown('观测与执行指标：等待 ROS 数据')
        # roslibpy loads Twisted/zope while connecting; create all Gradio
        # components first so that Gradio's component metaclass is unaffected.
        desk = Desk(args)
        views = [output, image, state, decision_view, candidates_view,
                 confidence_plot, metrics_view]
        analyze.click(desk.analyze, [instruction, reference], views)
        execute.click(desk.execute, [], views)
        cancel.click(desk.cancel_task, [], [output, state])
    demo.launch(server_name='127.0.0.1', server_port=args.server_port)


if __name__ == '__main__':
    main()
