#!/usr/bin/env python3
"""build_speaker_notes.py - one speaker-note PDF per slide (style of PPT/sampple speaker notes), via headless Chrome."""
import os, subprocess, html, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from slide_text import S
OUT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../PPT/speaker notes/'))
os.makedirs(OUT, exist_ok=True)
CSS = """body{font-family:'Liberation Sans',Arial,sans-serif;color:#1D2733;max-width:720px;margin:0 auto;font-size:13.5px;line-height:1.5}
h1{font-size:30px;margin:6px 0 6px}h2{font-size:21px;margin:26px 0 8px}.meta{color:#6A737D;font-size:12px;margin-bottom:10px}
.tag{background:#EEF1F4;border-radius:10px;padding:2px 9px;margin-right:6px}.script{border-left:3px solid #D5DBE1;padding-left:16px}
li{margin:4px 0}.q{margin:8px 0}@page{size:A4;margin:16mm}"""
for i, d in enumerate(S, 1):
    h = ['<html><head><meta charset="utf-8"><style>%s</style></head><body>' % CSS]
    h.append('<div class="meta">Speaker file %d · %s</div><h1>%s</h1>' % (i, html.escape(d['title']), html.escape(d['title'])))
    h.append('<div class="meta"><span class="tag">Nov 10, 2026</span><span class="tag">%s</span><span class="tag">about %d seconds</span></div>' % (html.escape(d['speaker']), d['secs']))
    h.append('<h2>One-minute script</h2><div class="script">' + ''.join('<p>%s</p>' % html.escape(p) for p in d['script']) + '</div>')
    h.append('<h2>What you need to understand</h2>')
    ul = []
    for p in d['understand']:
        if p.startswith('- '): ul.append('<li>%s</li>' % html.escape(p[2:]))
        else:
            if ul: h.append('<ul>' + ''.join(ul) + '</ul>'); ul = []
            h.append('<p>%s</p>' % html.escape(p))
    if ul: h.append('<ul>' + ''.join(ul) + '</ul>')
    h.append('<h2>Likely questions</h2>' + ''.join('<p class="q"><b>%s</b> %s</p>' % (html.escape(q), html.escape(a)) for q, a in d['questions']))
    h.append('</body></html>')
    base = 'Speaker file %d – %s' % (i, d['title'].replace(':', ' -').replace('→', 'to').replace('/', '-'))
    hp = '/tmp/claude-1000/-home-arjun/52b76a63-25b5-427a-93fa-4c2d507906ad/scratchpad/sn_%d.html' % i
    open(hp, 'w').write(''.join(h))
    subprocess.run(['google-chrome', '--headless', '--no-sandbox', '--disable-gpu', '--no-pdf-header-footer', '--print-to-pdf=%s/%s.pdf' % (OUT, base), 'file://' + hp], check=True, capture_output=True)
    print(base)
