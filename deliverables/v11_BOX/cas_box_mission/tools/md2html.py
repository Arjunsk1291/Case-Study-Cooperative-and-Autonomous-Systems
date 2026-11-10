#!/usr/bin/env python3
"""md2html.py SRC.md DST.html - tiny Markdown -> HTML (headings, paragraphs, lists, tables, images, code) for the report PDFs (print with headless Chrome)."""
import sys, re, html
src, dst = sys.argv[1], sys.argv[2]
CSS = "body{font-family:'Liberation Sans',Arial,sans-serif;color:#1D2733;max-width:900px;margin:0 auto;font-size:13px;line-height:1.5}h1{font-size:26px}h2{font-size:19px;margin-top:22px}table{border-collapse:collapse;width:100%;margin:8px 0}td,th{border:1px solid #d5dbe1;padding:4px 7px;vertical-align:top;font-size:12px}th{background:#eef1f4}code{background:#eef1f4;padding:0 3px}pre{background:#eef1f4;padding:8px}img{max-width:100%}@page{size:A4;margin:14mm}"
def inl(t):
    t = html.escape(t); t = re.sub(r'`([^`]+)`', r'<code>\1</code>', t); t = re.sub(r'\*\*([^*]+)\*\*', r'<b>\1</b>', t); return re.sub(r'\*([^*]+)\*', r'<i>\1</i>', t)
out = ['<html><head><meta charset="utf-8"><style>%s</style></head><body>' % CSS]; L = open(src).read().split('\n'); i = 0
while i < len(L):
    l = L[i]
    if l.startswith('```'):
        j = i + 1; b = []
        while not L[j].startswith('```'): b.append(L[j]); j += 1
        out.append('<pre>' + html.escape('\n'.join(b)) + '</pre>'); i = j + 1; continue
    m = re.match(r'!\[(.*)\]\((.*)\)', l)
    if m: out.append('<p><img src="%s" alt="%s"></p>' % (m.group(2), m.group(1))); i += 1; continue
    if l.startswith('#'):
        n = len(l) - len(l.lstrip('#')); out.append('<h%d>%s</h%d>' % (n, inl(l[n:].strip()), n)); i += 1; continue
    if l.startswith('|'):
        rows = []
        while i < len(L) and L[i].startswith('|'): rows.append([c.strip() for c in L[i].strip().strip('|').split('|')]); i += 1
        out.append('<table>' + ''.join('<tr>' + ''.join(('<th>' if k == 0 else '<td>') + inl(c) + ('</th>' if k == 0 else '</td>') for c in r) + '</tr>' for k, r in enumerate(rows) if k != 1) + '</table>'); continue
    if l.startswith('- '):
        its = []
        while i < len(L) and L[i].startswith('- '): its.append(inl(L[i][2:])); i += 1
        out.append('<ul>' + ''.join('<li>%s</li>' % x for x in its) + '</ul>'); continue
    if l.strip(): out.append('<p>%s</p>' % inl(l))
    i += 1
out.append('</body></html>'); open(dst, 'w').write('\n'.join(out))
