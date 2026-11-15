#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Optional isolated Python 3.7+ online speech input. Disabled by default.

Uses the official SDK transport and signing helper. This transport override
retains certificate validation, which the inspected SDK's default disables.
No ROS Python modules are imported into the speech interpreter.
"""
import argparse
import contextlib
import io
import json
import os
import pathlib
import select
import signal
import ssl
import subprocess
import sys
import time


class Transcript:
    def __init__(self):
        self.parts = {}
        self.final = False

    def accept(self, data):
        result = data.get('result', {})
        if result:
            serial = int(result['sn'])
            if result.get('pgs') == 'rpl':
                start, end = result['rg']
                for key in list(self.parts):
                    if start <= key <= end:
                        del self.parts[key]
            self.parts[serial] = ''.join(word['cw'][0]['w'] for word in result.get('ws', []) if word.get('cw'))
        self.final = self.final or data.get('status') == 2

    @property
    def text(self):
        return ''.join(self.parts[key] for key in sorted(self.parts))


def recognize(pcm, credentials, live_stream=None):
    from xfyunsdkspeech.iat_client import IatClient, _IatClient
    from xfyunsdkcore.signature import Signature
    import websocket

    class VerifiedTransport(_IatClient):
        def run(self, param, stream, frame_size):
            try:
                connection = websocket.WebSocketApp(
                    Signature.create_signed_url(self.api_url, self.api_key, self.api_secret),
                    on_message=self.on_message, on_error=self.on_error,
                    on_close=self.on_close, on_open=self.on_open)
                self.connection = connection
                connection.param, connection.stream, connection.frame_size = param, stream, frame_size
                connection.run_forever(sslopt={'cert_reqs': ssl.CERT_REQUIRED}, ping_interval=15, ping_timeout=5)
            except Exception as exc:
                self.queue.put({'error': str(exc), 'error_code': -1})

    options = {'vad_eos': 600} if live_stream is not None else {}
    client = IatClient(app_id=credentials['XF_APP_ID'], api_key=credentials['XF_API_KEY'],
                       api_secret=credentials['XF_API_SECRET'], dwa='wpgs', ptt=0, request_timeout=30, **options)
    client.client = VerifiedTransport(credentials['XF_APP_ID'], credentials['XF_API_KEY'], credentials['XF_API_SECRET'])
    transcript = Transcript()
    try:
        for chunk in client.stream(live_stream if live_stream is not None else io.BytesIO(pcm)):
            transcript.accept(chunk)
    finally:
        if live_stream is not None:
            live_stream.close()
            connection = getattr(client.client, 'connection', None)
            if connection is not None:
                connection.close()
    if not transcript.final or (live_stream is None and not transcript.text.strip()):
        raise RuntimeError('No complete final transcript; nothing was submitted')
    return transcript.text


class LiveAudio:
    """arecord byte stream; each normal SDK session ends within 50 seconds."""
    def __init__(self, device):
        self.deadline = time.monotonic() + 50.
        self.process = subprocess.Popen(['arecord', '-q', '-D', device, '-t', 'raw',
                                         '-f', 'S16_LE', '-r', '16000', '-c', '1'],
                                        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        self.closed = False

    def read(self, size):
        data = bytearray()
        while not self.closed and len(data) < size:
            remaining = self.deadline - time.monotonic()
            if remaining <= 0:
                break
            if not select.select([self.process.stdout], [], [], min(remaining, 2.))[0]:
                raise RuntimeError('Recording stalled')
            chunk = os.read(self.process.stdout.fileno(), size - len(data))
            if not chunk:
                raise RuntimeError('Recording device closed')
            data.extend(chunk)
        return bytes(data)

    def close(self):
        self.closed = True
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.process.stdout.close()


def continuous(credentials, device):
    def terminate(signum, frame):
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, terminate)
    while True:
        audio = LiveAudio(device)
        try:
            print(json.dumps({'event': 'listening'}), flush=True)
            # Keep SDK diagnostics off the JSON channel and out of ROS logs.
            with contextlib.redirect_stdout(sys.stderr):
                text = recognize(None, credentials, live_stream=audio)
            if text.strip():
                print(json.dumps({'final_text': text}, ensure_ascii=False), flush=True)
        finally:
            audio.close()


def main():
    parser = argparse.ArgumentParser(description='Speech transcription; continuous input is used by mode 3.')
    parser.add_argument('--pcm', type=pathlib.Path, help='16 kHz mono signed 16-bit little-endian raw PCM')
    parser.add_argument('--seconds', type=int, default=5, help='Record with arecord when --pcm is absent')
    parser.add_argument('--print-only', action='store_true')
    parser.add_argument('--continuous', action='store_true', help='Mode 3 live stream; JSON output only')
    parser.add_argument('--device', default='pulse', help='ALSA capture device for continuous input')
    parser.add_argument('--credentials', type=pathlib.Path, help='Private mode 3 dotenv file (owner-only)')
    args = parser.parse_args()
    if not 1 <= args.seconds <= 60:
        parser.error('--seconds must be between 1 and 60')
    if args.continuous and (args.pcm or not args.print_only):
        parser.error('--continuous requires --print-only and cannot use --pcm')
    values = os.environ
    if args.credentials:
        if not args.continuous:
            parser.error('--credentials is only used by the new continuous input')
        if args.credentials.stat().st_mode & 0o077:
            parser.error('Credential file must have owner-only permissions (chmod 600)')
        from dotenv import dotenv_values
        values = dotenv_values(str(args.credentials))
    credentials = {key: values.get(key) for key in ['XF_APP_ID', 'XF_API_KEY', 'XF_API_SECRET']}
    if not all(credentials.values()):
        parser.error('Voice is not configured: set XF_APP_ID, XF_API_KEY and XF_API_SECRET in the isolated speech environment')
    if args.continuous:
        continuous(credentials, args.device)
        return
    if args.pcm:
        pcm = args.pcm.read_bytes()
    else:
        pcm = subprocess.check_output(['arecord', '-q', '-t', 'raw', '-f', 'S16_LE', '-r', '16000', '-c', '1', '-d', str(args.seconds)], timeout=args.seconds + 5)
    if not 0 < len(pcm) <= 60 * 16000 * 2 or len(pcm) % 2:
        parser.error('Audio must contain 1..60 seconds of complete 16-bit PCM samples')
    text = recognize(pcm, credentials)
    print(json.dumps({'final_text': text}, ensure_ascii=False))
    if not args.print_only:
        print('Transcription only; autonomous delivery routing has been removed.', file=sys.stderr)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        if '--continuous' in sys.argv:
            print(json.dumps({'event': 'error'}), flush=True)
            print('Continuous speech failed; check audio/account/network.', file=sys.stderr)
        else:
            print('Speech failed: ' + str(exc), file=sys.stderr)
        sys.exit(1)
