#!/usr/bin/env python3
"""build_report.py - fills report_template.html with the three SVG diagrams and the figures (embedded as data URIs)
-> BtoI_report.html (one file; the PDF is printed from it with headless Chrome)."""
import base64, os
HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, '../../PPT/new_slides_assets/')


def img(name):
    return 'data:image/png;base64,' + base64.b64encode(open(ASSETS + name, 'rb').read()).decode()


C = dict(blue='var(--blue)', orange='var(--orange)', ink='var(--ink)', mute='var(--mute)', panel='var(--panel)', line='var(--line)', soft='var(--blue-soft)', osoft='var(--orange-soft)')


def box(x, y, w, h, title, sub='', stroke='blue', fill='panel', tsize=13):
    t = f'<text x="{x + w/2}" y="{y + (h/2 - 2 if sub else h/2 + 5)}" text-anchor="middle" font-size="{tsize}" font-weight="600" fill="{C["ink"]}">{title}</text>'
    s = f'<text x="{x + w/2}" y="{y + h/2 + 14}" text-anchor="middle" font-size="11" fill="{C["mute"]}">{sub}</text>' if sub else ''
    return f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="{C[fill]}" stroke="{C[stroke]}" stroke-width="1.4"/>' + t + s


def tbox(x, y, w, h, title, lines, stroke='blue'):
    out = f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="{C["panel"]}" stroke="{C[stroke]}" stroke-width="1.4"/>'
    out += f'<text x="{x + w/2}" y="{y + 26}" text-anchor="middle" font-size="14" font-weight="600" fill="{C["ink"]}">{title}</text>'
    for i, l in enumerate(lines):
        out += f'<text x="{x + w/2}" y="{y + 56 + i*22}" text-anchor="middle" font-size="11.5" fill="{C["mute"]}">{l}</text>'
    return out


def arrow(x1, y1, x2, y2, color='mute', dash=False, label='', lx=0, ly=0):
    d = ' stroke-dasharray="5 4"' if dash else ''
    s = f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{C[color]}" stroke-width="1.6"{d} marker-end="url(#ah-{color})"/>'
    if label: s += f'<text x="{lx or (x1+x2)/2}" y="{ly or (y1+y2)/2 - 5}" text-anchor="middle" font-size="10.5" fill="{C[color]}">{label}</text>'
    return s


DEFS = ('<defs>' + ''.join(f'<marker id="ah-{k}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
                          f'<path d="M0 0L10 5L0 10z" fill="{C[k]}"/></marker>' for k in ('mute', 'blue', 'orange')) + '</defs>')


def route_svg():
    s = ['<svg viewBox="0 0 980 430" role="img" aria-label="Route from B to Block I with the landmark of every leg">', DEFS]
    # legs: L1 north along x=110, L2 east along y=120, L3 south along x=640, L4 east along y=250
    s.append(f'<polyline points="110,390 110,120 640,120 640,250 930,250" fill="none" stroke="{C["blue"]}" stroke-width="5" stroke-linejoin="round" stroke-linecap="round" opacity=".28"/>')
    s.append(f'<polyline points="110,390 110,120 640,120 640,250 930,250" fill="none" stroke="{C["blue"]}" stroke-width="2" stroke-dasharray="1 9" stroke-linecap="round"/>')
    # B facade (left of leg 1 = west)
    s.append(f'<rect x="62" y="130" width="12" height="255" fill="{C["mute"]}" opacity=".5"/><text x="46" y="258" text-anchor="middle" font-size="12" fill="{C["mute"]}" transform="rotate(-90 46 258)">B facade (wall on the left, 1.1 to 1.6 m)</text>')
    s.append(f'<rect x="62" y="195" width="12" height="55" fill="{C["panel"]}" stroke="none"/><text x="86" y="226" font-size="10.5" fill="{C["mute"]}">gap</text>')
    for yy in (360, 315, 285, 175):
        s.append(f'<circle cx="148" cy="{yy}" r="6" fill="{C["mute"]}"/>')
    s.append(f'<rect x="136" y="118" width="24" height="24" rx="3" fill="{C["blue"]}" opacity=".85"/><text x="170" y="150" font-size="12" font-weight="600" fill="{C["blue"]}">THD sign structure</text><text x="170" y="165" font-size="12" font-weight="600" fill="{C["blue"]}">+ turn pillar</text>')
    s.append(f'<text x="160" y="360" font-size="11" fill="{C["mute"]}">pillars (right side)</text>')
    s.append(f'<text x="128" y="395" font-size="13" font-weight="600" fill="{C["ink"]}">START</text>')
    s.append(f'<text x="116" y="106" font-size="12" font-weight="600" fill="{C["orange"]}">turn 1: right, 0.4 m past the pillar edge</text>')
    s.append(f'<text x="122" y="300" font-size="12" font-weight="600" fill="{C["blue"]}" transform="rotate(-90 122 300)" text-anchor="middle">LEG 1  ~20 m</text>')
    # open sea landmarks (north of leg 2)
    s.append(f'<line x1="280" y1="60" x2="440" y2="60" stroke="{C["blue"]}" stroke-width="3"/>' + ''.join(f'<circle cx="{x}" cy="60" r="5" fill="{C["blue"]}"/>' for x in range(285, 440, 32)))
    s.append(f'<text x="360" y="44" text-anchor="middle" font-size="12" font-weight="600" fill="{C["blue"]}">tree / post row, 6.65 m left (10 to 24.5 m)</text>')
    s.append(f'<rect x="470" y="86" width="120" height="10" fill="{C["blue"]}" opacity=".7"/><text x="530" y="78" text-anchor="middle" font-size="12" font-weight="600" fill="{C["blue"]}">building front, 3.0 m</text>')
    s.append(f'<rect x="186" y="95" width="6" height="50" fill="{C["blue"]}" opacity=".0"/>')
    s.append(f'<text x="210" y="190" font-size="11" fill="{C["mute"]}">start of leg 2: B wall behind,</text><text x="210" y="204" font-size="11" fill="{C["mute"]}">perpendicular to it</text>')
    s.append(f'<text x="470" y="146" text-anchor="middle" font-size="12" font-weight="600" fill="{C["blue"]}">LEG 2  OPEN SEA  ~40 m  (tactile strip)</text>')
    s.append(f'<rect x="652" y="108" width="8" height="26" fill="{C["ink"]}"/><text x="665" y="106" font-size="12" font-weight="600" fill="{C["ink"]}">end wall</text>')
    s.append(f'<text x="560" y="180" font-size="12" font-weight="600" fill="{C["orange"]}">turn 2: right</text>')
    # arcade
    s.append(f'<rect x="670" y="128" width="10" height="110" fill="{C["mute"]}" opacity=".5"/><text x="690" y="190" font-size="12" font-weight="600" fill="{C["blue"]}">LEG 3  arcade wall</text><text x="690" y="206" font-size="11" fill="{C["mute"]}">left, 0.6 m, ~10 m</text>')
    s.append(f'<text x="560" y="272" font-size="12" font-weight="600" fill="{C["orange"]}">T3: left, wall ends</text>')
    # leg 4
    s.append(f'<text x="700" y="318" font-size="12" font-weight="600" fill="{C["blue"]}">LEG 4  wall 1.0 m, then interlock walkway</text>')
    s.append(f'<rect x="640" y="215" width="150" height="8" fill="{C["mute"]}" opacity=".45"/>')
    s.append(f'<line x1="790" y1="258" x2="905" y2="258" stroke="{C["blue"]}" stroke-width="3"/><line x1="790" y1="236" x2="905" y2="236" stroke="{C["mute"]}" stroke-width="3" opacity=".6"/>')
    s.append(f'<text x="848" y="283" text-anchor="middle" font-size="11" fill="{C["blue"]}">railings (right 0.5 to 1 m)</text>')
    s.append(f'<rect x="866" y="222" width="52" height="8" fill="{C["orange"]}" opacity=".8"/><text x="893" y="216" text-anchor="middle" font-size="11" font-weight="600" fill="{C["orange"]}">KARL entrance</text>')
    s.append(f'<circle cx="930" cy="250" r="9" fill="{C["orange"]}"/><text x="930" y="278" text-anchor="middle" font-size="13" font-weight="700" fill="{C["orange"]}">STOP (I)</text>')
    s.append('</svg>')
    return ''.join(s)


def arch_svg():
    s = ['<svg viewBox="0 0 980 360" role="img" aria-label="Data flow in the box">', DEFS]
    s.append(box(10, 20, 170, 56, 'LD20 LiDAR', '90 bins x 4 deg, 4 Hz'))
    s.append(box(10, 90, 170, 56, 'BNO055 IMU', 'yaw, pitch, roll'))
    s.append(box(10, 160, 170, 56, '5 x HC-SR04', 'front, 45 deg down, sides'))
    s.append(tbox(240, 20, 190, 196, 'Sensors', ['LidarScan: 35 cm minimum', 'intensity per beam', 'UsManager: 60 ms pings', 'ImuManager: checked reads']))
    for y in (48, 118, 188): s.append(arrow(180, y, 240, y, 'mute'))
    s.append(tbox(500, 20, 230, 196, 'RouteMission', ['state machine, legs, turns', 'WallFit, LineRef (RANSAC)', 'turn structure, landmark corridor', 'leg 4: corridor, entrance', 'DriveStraight: gyro hold']))
    s.append(arrow(430, 118, 500, 118, 'blue', label='scan'))
    s.append(box(790, 20, 180, 56, 'GentleDrive', 'every command is a ramp', stroke='orange'))
    s.append(arrow(730, 48, 790, 48, 'blue', label='l, r'))
    s.append(box(790, 100, 180, 56, 'RoverLink', 'HTTP JSON T:1, 10 Hz'))
    s.append(arrow(880, 76, 880, 100, 'blue'))
    s.append(box(790, 180, 180, 56, 'WAVE ROVER', 'own firmware, unchanged', fill='panel'))
    s.append(arrow(880, 156, 880, 180, 'blue'))
    s.append(arrow(790, 208, 730, 208, 'mute', dash=True, label='T:130 (read)', lx=760, ly=200))
    s.append(box(240, 270, 190, 56, 'MissionLog', '249-byte records, 4 Hz'))
    s.append(box(500, 270, 230, 56, 'Phone page (WebHMI)', 'STOP, manual, climb, speed +/-'))
    s.append(arrow(615, 216, 615, 270, 'mute', dash=True, label='status, commands', lx=690, ly=246))
    s.append(arrow(430, 298, 500, 298, 'mute', dash=True))
    s.append(arrow(335, 216, 335, 270, 'mute', dash=True))
    s.append('</svg>')
    return ''.join(s)


def state_svg():
    s = ['<svg viewBox="0 0 980 400" role="img" aria-label="State machine of the mission">', DEFS]
    row = [(10, 'IDLE', 'Start pressed'), (150, 'L1_FOLLOW', 'left wall, turn structure'), (320, 'TURN + SETTLE', 'pivot 90 deg, check'),
           (490, 'L2_GO', 'open sea corridor'), (640, 'TURN', 'right at end wall'), (790, 'L3_FOLLOW', 'arcade wall 0.6 m')]
    for x, t, sub in row:
        s.append(box(x, 40, 130 if t != 'TURN + SETTLE' else 140, 58, t, sub, stroke='blue' if 'TURN' not in t else 'orange', tsize=12.5))
    s.append(arrow(140, 69, 150, 69, 'blue')); s.append(arrow(280, 69, 320, 69, 'blue')); s.append(arrow(460, 69, 490, 69, 'blue'))
    s.append(arrow(620, 69, 640, 69, 'blue')); s.append(arrow(770, 69, 790, 69, 'blue'))
    s.append(box(790, 150, 130, 58, 'TURN (T3)', 'left, wall ended', stroke='orange', tsize=12.5))
    s.append(arrow(855, 98, 855, 150, 'blue', label='wall ends', lx=895, ly=128))
    s.append(box(560, 150, 190, 58, 'L4_FOLLOW', 'wall, walkway, railings, top', tsize=12.5))
    s.append(arrow(790, 179, 750, 179, 'blue'))
    s.append(box(560, 260, 190, 50, 'DONE / END_ASK', 'ARRIVED, gentle stop', stroke='orange', tsize=12.5))
    s.append(arrow(655, 208, 655, 260, 'blue', label='entrance + 1.5 m', lx=720, ly=240))
    # interrupts
    s.append(box(60, 200, 150, 54, 'HOLD', 'something in front: stop, wait', stroke='orange', tsize=12.5))
    s.append(box(240, 200, 150, 54, 'PAUSED', 'STOP pressed, real tilt', stroke='orange', tsize=12.5))
    s.append(box(420, 200, 120, 54, 'MANUAL', 'pad touched, climb', stroke='orange', tsize=12.5))
    s.append(f'<text x="60" y="290" font-size="12" fill="{C["ink"]}">Any moving state can go to these three and come back:</text>')
    s.append(f'<text x="60" y="310" font-size="11.5" fill="{C["mute"]}">HOLD ends after 2 s clear (> 1 m) and valid beams, retries after 20 s (3 times).</text>')
    s.append(f'<text x="60" y="328" font-size="11.5" fill="{C["mute"]}">PAUSED and MANUAL need a button. Link lost: stand still and go on. Turn not done in 2 min: try again.</text>')
    s.append(f'<text x="60" y="352" font-size="11.5" fill="{C["mute"]}">No sign structure found by the end of its window: T1_ASK, the phone asks to turn (no blind turn).</text>')
    s.append('</svg>')
    return ''.join(s)


if __name__ == '__main__':
    t = open(HERE + '/report_template.html').read()
    t = t.replace('@@ROUTE_SVG@@', route_svg()).replace('@@ARCH_SVG@@', arch_svg()).replace('@@STATE_SVG@@', state_svg())
    for tag, f in (('TURN', 'fig_turn.png'), ('SEA', 'fig_sea.png'), ('BLOCKI', 'fig_blockI.png'), ('PEOPLE', 'fig_people.png')):
        t = t.replace(f'@@IMG_{tag}@@', img(f))
    t = t.replace('B to I: 20 m wall, 40 m open sea, 10 m arcade, 60 m interlock and railings', 'B to I: 20 m wall, 40 m open sea, 10 m arcade, about 50 m interlock and railings')
    t = t.replace('<b>~160 m</b>', '<b>~120 m</b>').replace('<b>10 logs</b>', '<b>9 logs</b>').replace(' Loop under 10 ms.', '')
    open(HERE + '/BtoI_report.html', 'w').write(t)
    print('written', len(t) // 1024, 'kB')
