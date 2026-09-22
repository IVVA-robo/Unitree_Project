from pathlib import Path


WEB_ROOT = Path(__file__).resolve().parents[1] / 'web'


def test_viewer_is_fully_local_and_has_fallbacks():
    html = (WEB_ROOT / 'index.html').read_text(encoding='utf-8')
    javascript = (WEB_ROOT / 'app.js').read_text(encoding='utf-8')
    combined = html + javascript
    assert 'https://' not in combined
    assert 'http://' not in combined
    assert '/offer' in javascript
    assert '/stream.mjpg' in javascript
    assert 'top-bottom' in combined
    assert 'immersive' in javascript.lower() or 'immersive' in combined.lower()
    assert 'overlay-hidden' in combined
    assert 'resolutionValue' in combined
    assert 'droppedValue' in combined
    assert 'webxrValue' in combined
    assert 'iceServers: []' in javascript
    assert "query.get('transport')" in javascript
    assert 'serverFresh && navigator.onLine' not in javascript
    assert "downgradeProfile('повторные переподключения', true)" in javascript
    assert 'NO SIGNAL' in html or 'no-signal' in html


def test_viewer_has_no_robot_control_endpoint():
    contents = ''.join(
        path.read_text(encoding='utf-8')
        for path in WEB_ROOT.iterdir()
        if path.is_file()
    ).lower()
    assert 'cmd_vel' not in contents
    assert 'joint_trajectory' not in contents
    assert '/vr/' not in contents
