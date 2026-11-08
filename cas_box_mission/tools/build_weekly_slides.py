#!/usr/bin/env python3
"""build_weekly_slides.py - three weekly-progress slides in the team template (PPT/CAS rover - local path tracking ...).

Starts from the team's own deck (same master, layout, fonts, colours), removes its slides and builds the new ones with
the same shapes: grey eyebrow, bold 32 pt title, white boxes with blue / orange outline, footer (topic left, name right).
Needs python-pptx:  PYTHONPATH=<dir with python-pptx> python3 build_weekly_slides.py
"""
import copy, os, sys
from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.dml import MSO_THEME_COLOR
from pptx.oxml.ns import qn
from lxml import etree

HERE = os.path.dirname(os.path.abspath(__file__))
PPT = os.path.join(HERE, '../../PPT/')
ASSETS = PPT + 'new_slides_assets/'
BASE = PPT + 'CAS rover – local path tracking (log_020–022) (1).pptx'
OUT = PPT + 'CAS rover – landmark corridor, people simulation, Block I (week 3).pptx'
INK, BODY, GREY, BLUE, ORANGE, ORANGE_T, LINE = '1D2733', '3D4853', '6A737D', '1F6FB2', 'E07B1A', 'C0620C', 'D5DBE1'
NAME = 'Arjun'


def rgb(h): return RGBColor.from_string(h)


def style_run(r, size, color, bold=False, spc=None, theme=None):
    r.font.name = 'Arial'; r.font.size = Pt(size); r.font.bold = bold
    if theme: r.font.color.theme_color = theme
    else: r.font.color.rgb = rgb(color)
    rPr = r._r.get_or_add_rPr()
    for tag in ('a:ea', 'a:cs'):
        e = rPr.find(qn(tag))
        if e is None: e = etree.SubElement(rPr, qn(tag))
        e.set('typeface', 'Arial')
    if spc is not None: rPr.set('spc', str(spc))


def text(slide, x, y, w, h, paras, size=12, color=BODY, bold=False, align=PP_ALIGN.LEFT, bullets=False, spc=None, line=None,
         anchor=MSO_ANCHOR.TOP, theme=None):
    """paras: list of str | list of (str, dict(bold, color)) runs"""
    tb = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = tb.text_frame; tf.word_wrap = True; tf.vertical_anchor = anchor
    tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
    for i, para in enumerate(paras):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.alignment = align
        p.line_spacing = Pt(line if line else size * 1.35)
        if bullets:
            pPr = p._p.get_or_add_pPr(); pPr.set('marL', '142494'); pPr.set('indent', '-142494')
            etree.SubElement(pPr, qn('a:buSzPct')).set('val', '100000'); etree.SubElement(pPr, qn('a:buChar')).set('char', '•')
        runs = para if isinstance(para, list) else [(para, {})]
        for t, o in runs:
            r = p.add_run(); r.text = t
            style_run(r, o.get('size', size), o.get('color', color), o.get('bold', bold), spc, theme)
    return tb


def box(slide, x, y, w, h, edge=BLUE, radius=0.045, fill='FFFFFF', width=1.0):
    s = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(x), Inches(y), Inches(w), Inches(h))
    s.adjustments[0] = radius
    s.fill.solid(); s.fill.fore_color.rgb = rgb(fill)
    s.line.color.rgb = rgb(edge); s.line.width = Pt(width)
    st = s._element.find(qn('p:style'))
    if st is not None: s._element.remove(st)                  # no theme shadow / effects
    return s


def card(slide, x, y, w, h, head, lines, edge=BLUE, head_color=None, size=12):
    """white box + bold heading + bullet list (the team's box)"""
    box(slide, x, y, w, h, edge)
    hc = head_color or (BLUE if edge == BLUE else ORANGE_T)
    text(slide, x + 0.2, y + 0.13, w - 0.35, 0.3, [head], size=14, color=hc, bold=True)
    text(slide, x + 0.2, y + 0.5, w - 0.35, h - 0.6, lines, size=size, color=BODY, bullets=True, line=size * 1.28)


def picture(slide, path, x, y, w, h=None):
    pic = slide.shapes.add_picture(path, Inches(x), Inches(y), Inches(w), Inches(h) if h else None)
    pic.auto_shape_type = MSO_SHAPE.ROUNDED_RECTANGLE
    av = pic._element.spPr.find(qn('a:prstGeom')).find(qn('a:avLst'))
    for g in list(av): av.remove(g)
    gd = etree.SubElement(av, qn('a:gd')); gd.set('name', 'adj'); gd.set('fmla', 'val 1833')
    fr = box(slide, x, y, Emu(pic.width).inches, Emu(pic.height).inches, edge=LINE, radius=0.019, fill='FFFFFF', width=0.5)
    fr.fill.background()                                   # frame on top, no fill
    return pic


def chain(slide, x, y, items, bw, gap, bh=0.78, edge=BLUE, last_edge=None):
    """row of blocks with small arrows (like the control-loop strip of the team deck)"""
    for i, (name, sub) in enumerate(items):
        bx = x + i * (bw + gap)
        e = last_edge if (last_edge and i == len(items) - 1) else edge
        box(slide, bx, y, bw, bh, e, radius=0.1)
        text(slide, bx + 0.05, y + 0.1, bw - 0.1, 0.25, [name], size=12, color=INK, bold=True, align=PP_ALIGN.CENTER)
        text(slide, bx + 0.05, y + 0.38, bw - 0.1, 0.38, [sub], size=10, color=BODY, align=PP_ALIGN.CENTER, line=11.5)
        if i < len(items) - 1:
            a = slide.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW, Inches(bx + bw + gap * 0.18), Inches(y + bh / 2 - 0.06), Inches(gap * 0.64), Inches(0.12))
            a.fill.solid(); a.fill.fore_color.rgb = rgb('9AA3AD'); a.line.fill.background()
            st = a._element.find(qn('p:style'))
            if st is not None: a._element.remove(st)


def set_notes(slide, txt, body):
    ns = slide.notes_slide
    if ns.notes_text_frame is None:
        ns.shapes._spTree.append(copy.deepcopy(body))
    tf = ns.notes_text_frame
    tf.text = ''
    for i, para in enumerate(txt.split('\n\n')):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.text = para.strip()


def frame_slide(prs, eyebrow, title, footer, name=None):
    layout = prs.slide_layouts.get_by_name('Title and body')
    s = prs.slides.add_slide(layout)
    for ph in list(s.placeholders):
        if ph.placeholder_format.type is not None and 'TITLE' not in str(ph.placeholder_format.type):
            ph._element.getparent().remove(ph._element)
    text(s, 0.889, 0.667, 11.9, 0.34, [eyebrow], size=16, color=GREY, spc=100, line=22.4)
    t = s.shapes.title
    t.left, t.top, t.width, t.height = Inches(0.889), Inches(1.172), Inches(11.9), Inches(0.52)
    tf = t.text_frame; tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
    p = tf.paragraphs[0]; p.alignment = PP_ALIGN.LEFT; p.line_spacing = Pt(35.2)
    r = p.add_run(); r.text = title; style_run(r, 32, INK, bold=True)
    text(s, 0.889, 6.87, 7.0, 0.26, [footer], size=12, color=GREY, line=16.8)
    text(s, 9.4, 6.87, 3.1, 0.26, [name or NAME], size=12, bold=True, align=PP_ALIGN.RIGHT, line=16.8, theme=MSO_THEME_COLOR.ACCENT_4)
    return s


def notes_text(i):
    from slide_text import S
    d = S[i]
    t = 'SPEAKER: %s · about %d seconds\n\nONE-MINUTE SCRIPT\n\n' % (d['speaker'], d['secs']) + '\n\n'.join(d['script'])
    t += '\n\nWHAT YOU NEED TO UNDERSTAND\n\n' + '\n\n'.join(d['understand'])
    t += '\n\nLIKELY QUESTIONS\n\n' + '\n\n'.join('Q: %s\nA: %s' % qa for qa in d['questions'])
    return t


def build(N):
    prs = Presentation(BASE)
    # the deck's notes master has no text box: take the notes body placeholder of one of the team's own slides
    NOTE_BODY = None
    for sl in prs.slides:
        if sl.has_notes_slide:
            for sh in sl.notes_slide.shapes:
                if sh.is_placeholder and 'BODY' in str(sh.placeholder_format.type):
                    NOTE_BODY = copy.deepcopy(sh._element)
            if NOTE_BODY is not None: break
    lst = prs.slides._sldIdLst
    for sid in list(lst):
        prs.part.drop_rel(sid.rId); lst.remove(sid)

    # ================================================================ slide 1: landmark corridor
    s = frame_slide(prs, 'OPEN SEA · LANDMARK CORRIDOR', 'Landmark proximity corridor for the Open Sea', 'Landmark corridor', 'Arjun')
    picture(s, ASSETS + 'fig_sea.png', 0.889, 1.86, 7.8)
    chain(s, 0.889, 4.93, [('B wall behind', 'heading ⟂ · 0–14 m'), ('Store front + poles', 'side 6.65 m · 10–24.5 m'),
                           ('Building front', 'side 3.0 m · 26–40 m'), ('End wall ahead', 'heading ⟂ · stop 1.5 m')], 1.72, 0.30, 0.74)
    box(s, 0.889, 5.88, 7.8, 0.82, BLUE, radius=0.08)
    text(s, 1.05, 5.95, 7.5, 0.7, [[('Why landmarks: ', {'bold': True, 'color': INK}),
                                     ('in the Open Sea there are no walls to match. The gyro drifts by about ±2°/min at random, GPS was 6–15 m off (log_010). '
                                      'The U-shaped store front (with its two poles and two trees in front) is a fixed line, so it gives the side position directly.', {})]],
         size=12, line=15.5)
    card(s, 9.0, 1.86, 3.85, 1.38, 'Proximity PID on the store front', [
        'e = measured line distance − 6.65 m',
        'turn = kp·e + ki·∫e + kd·ė  (10 / 1.5 / 20, ±8°)',
        'gate ±1.0 m (v10.1): false lines are rejected'], size=12)
    card(s, 9.0, 3.32, 3.85, 1.46, 'Measured in 9 logs', [
        f'store-front distance {N["row_lo"]}–{N["row_hi"]} m in 5 of 9 runs; teach run 6.67 m',
        'seen on 39–56 % of the scans',
        'B wall behind: ⟂ within ±1.3° in 6 runs'], size=12)
    card(s, 9.0, 4.88, 3.85, 1.82, 'Result', [
        'log8, campus_9: wall to wall, no hold, pause or failsafe',
        'heading changed 5.6° over 49 m and 5.8° over 42 m',
        'fault sim: false line 1.6–2.0 m off → 0.01–0.03 m'], edge=ORANGE, size=11)
    set_notes(s, notes_text(0), NOTE_BODY)

    # ================================================================ slide 2: Open Sea simulation
    s = frame_slide(prs, 'SIMULATION · OPEN SEA', 'Open Sea simulation: crowds, weather and faults', 'Open Sea simulation', 'Kamal')
    chain(s, 0.889, 1.86, [('World model', 'B wall, store front, building, end wall'), ('Scanner', '90 beams, 2 cm noise, 12 % drops'),
                           ('Rover model', '0.22 m/s, slide, gyro bias'), ('Firmware mirror', 'same rules, 0.25 s step'),
                           ('Metrics', 'side error, false end, holds')], 2.0, 0.40, 0.78)
    picture(s, ASSETS + 'fig_faults.png', 0.889, 2.84, 7.4)
    text(s, 0.889, 6.3, 7.6, 0.45, ['tools/sim_open_sea.py + tools/fwmirror.py (class OpenSea). 20 random starts per case, 12 for weather. v10.1 = row gate ±1.0 m.'], size=11, color=GREY, line=14)
    card(s, 8.75, 2.84, 4.1, 1.5, 'Crowds (20 starts each)', [
        'one person in the path: false end wall 18 → 0',
        'group of 3: 20 → 0; walking crowd of 20: 13 → 0',
        'rover stops and waits, never swerves'], size=11.5)
    card(s, 8.75, 4.42, 4.1, 1.2, 'Weather (12 starts)', [
        'ghosts, low sun, light rain: 12 / 12',
        'heavy rain / snow 10 / 12; worst mix 7 / 12'], size=11)
    card(s, 8.75, 5.7, 4.1, 1.0, 'Found and open', [
        'false line pulled it 1.6–2.0 m: fixed (gate)',
        'open: −15° glitch late, gyro bias with no landmark'], edge=ORANGE, size=10.5)
    set_notes(s, notes_text(1), NOTE_BODY)

    # ================================================================ slide 3: Block I
    s = frame_slide(prs, 'LEG 4 · BLOCK I', 'Last leg to Block I: corridor, entrance, stop', 'Last leg', '[name to be added]')
    picture(s, ASSETS + 'fig_blockI.png', 0.889, 1.86, 7.8)
    chain(s, 0.889, 4.93, [('Wall follow', '1.0 m, power 0.30'), ('Walkway', 'centre on left edge'), ('Railing corridor', 'width 2.4–3.3 m'),
                           ('KARL entrance', 'facade recess + glass'), ('Stop', 'entrance + 1.5 m')], 1.42, 0.175, 0.74, last_edge=ORANGE)
    box(s, 0.889, 5.88, 7.8, 0.82, ORANGE, radius=0.08)
    text(s, 1.05, 5.95, 7.5, 0.7, [[('campus_9 with v9: ', {'bold': True, 'color': ORANGE_T}),
                                     ('the rover passed the entrance and hit a bollard 9 m later. The corridor was not recognised (right railing 0.9 m, rule wanted ≤ 0.85 m) '
                                      'and the bollard at < 35 cm was below the LiDAR minimum range.', {})]], size=12, line=15.5)
    card(s, 9.0, 1.86, 3.85, 1.72, 'Walkway centering (not into the mud)', [
        'LiDAR line through the left edge: grass edge, benches, strip (1–3 m, ≥ 5 points)',
        'keeps the first-seen distance (1.3–2.4 m), 10°/m, max ±6°'], size=11)
    card(s, 9.0, 3.68, 3.85, 1.58, 'Railing corridor → KARL entrance', [
        'right line 0.35–1.3 m + left facade, width 2.4–3.3 m: found at 35.6 m',
        'facade steps back, glass drops beams: 47.0 m; stop at 48.5 m'], size=11)
    card(s, 9.0, 5.36, 3.85, 1.34, 'Limits (replay)', [
        'edge line found on only 10 % of the walkway scans in campus_9 (35 % in log8): the rest is heading hold',
        'edge assist at the cobble edge; any hold near the end finishes the route'], edge=ORANGE, size=10.5)
    set_notes(s, notes_text(2), NOTE_BODY)

    # ================================================================ slide 4: daylight ghost filter
    s = frame_slide(prs, 'SENSORS · DAYLIGHT GHOSTS', 'Daylight ghost filter: LiDAR and ultrasonic cross-check', 'Ghost filter', '[name to be added]')
    chain(s, 0.889, 1.86, [('LiDAR beam', 'ghosts 20–34 cm in sun'), ('Min range', 'drop < 35 cm'), ('Confirm', '≥ 4 beams or ultrasonic'),
                           ('Mask', 'floor echo 50–90 cm'), ('Act', 'gentle stop if real')], 2.0, 0.40, 0.78, last_edge=ORANGE)
    picture(s, ASSETS + 'fig_ghost.png', 0.889, 2.84, 7.4)
    text(s, 0.889, 6.3, 7.6, 0.45, ['Returns below 35 cm in the daytime logs recorded before the filter (campus_2, _3, _4, _5). Later logs contain none.'], size=11, color=GREY, line=14)
    card(s, 8.75, 2.84, 4.1, 1.5, 'From the datasheets', [
        'LD19 family: 0.02–12 m, 4,500 Hz, 30 klux ambient light',
        'HC-SR04: 2–400 cm, ~15° cone, ≥ 60 ms per ping'], size=11.5)
    card(s, 8.75, 4.42, 4.1, 1.2, 'Rule in the box', [
        'near obstacle: ≥ 4 valid beams, or front ultrasonic < 50 cm and ≤ LiDAR + 15 cm',
        'replay: 27 of 28 ghost events dropped'], size=10.5)
    card(s, 8.75, 5.7, 4.1, 1.0, 'Price', [
        'LiDAR is blind below 35 cm (bollard, campus_9)',
        'covered by ultrasonic stop at 35 cm + hold rule'], edge=ORANGE, size=10.5)
    set_notes(s, notes_text(3), NOTE_BODY)
    prs.save(OUT)
    print('saved', OUT)


if __name__ == '__main__':
    sys.path.insert(0, HERE)
    from slide_text import N
    build(N)
