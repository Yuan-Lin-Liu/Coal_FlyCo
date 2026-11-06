"""Verify authentication, startup waiting and wrong-service rejection."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import yaml
import pytest
from flyco_real.semantic_health import main, PROTOCOL, REVISION


@pytest.mark.parametrize('mode,expected', [('startup',0),('wrong_backend',1),('unauthorized',1)])
def test_semantic_readiness(tmp_path, monkeypatch, capsys, mode, expected):
    requests=[]
    token='semantic-test-token-with-no-real-credentials'
    monkeypatch.setenv('SAM_AD_API_TOKEN',token)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args): pass
        def do_GET(self):
            requests.append((self.path,self.headers.get('Authorization')))
            data={'protocol':PROTOCOL,'service_revision':REVISION,'backend':'A',
                  'status':'loading' if len(requests)==1 else 'ready'}
            if mode=='wrong_backend': data['backend']='D'
            payload=json.dumps(data).encode()
            self.send_response(401 if mode=='unauthorized' else 200)
            self.send_header('Content-Length',str(len(payload)))
            self.end_headers(); self.wfile.write(payload)
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
    thread=threading.Thread(target=server.serve_forever,daemon=True); thread.start()
    config=tmp_path/'vehicle.yaml'
    config.write_text(yaml.safe_dump({'sam_tracking':{'ros__parameters':{
        'segmentation_endpoint':'http://127.0.0.1:%d/api' % server.server_port}}}))
    try:
        assert main(['--config',str(config),'--wait','--timeout','3']) == expected
        assert len(requests) == (2 if mode=='startup' else 1)
        assert all(path=='/api/health' and auth=='Bearer '+token for path,auth in requests)
        assert token not in capsys.readouterr().out
    finally:
        server.shutdown(); server.server_close(); thread.join()
