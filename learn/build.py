# 학습서 템플릿에 코드·실행 로그 수치를 채워 index.html과 data.js를 만든다
import html
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
STEPS = os.path.join(HERE, "steps")
REPO = os.path.dirname(HERE)  # 저장소 루트
LOGS = os.path.join(HERE, "logs")  # 단계 스크립트 실행 로그


def hl(text):
    """이스케이프하고 주석만 색칠한다."""
    out = []
    for line in text.split("\n"):
        e = html.escape(line, quote=False)
        if line.lstrip().startswith("#"):
            e = f'<span class="c">{e}</span>'
        elif "  # " in e:
            a, b = e.split("  # ", 1)
            e = f'{a}  <span class="c"># {b}</span>'
        out.append(e)
    return "\n".join(out)


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def cut(text, start, end):
    lines = text.split("\n")
    i = next(k for k, l in enumerate(lines) if start in l)
    j = next(k for k in range(i, len(lines)) if end in lines[k])
    return "\n".join(lines[i : j + 1])


def fill(tpl):
    def code(m):
        name = m.group(1)
        return ('<div class="code"><div class="bar"><span>' + name + '</span><button class="copy" type="button">복사</button></div>'
                "<pre><code>" + hl(read(os.path.join(STEPS, name)).rstrip("\n")) + "</code></pre></div>")

    def snip(m):
        kind, name, start, end = m.group(1), m.group(2), m.group(3), m.group(4)
        base = STEPS if kind == "snippet" else REPO
        return hl(cut(read(os.path.join(base, name)), start, end))

    tpl = re.sub(r"\{\{code:([^}]+)\}\}", code, tpl)
    tpl = re.sub(r"\{\{(snippet|repo):([^:}]+):([^:}]+):([^}]+)\}\}", snip, tpl)
    assert "{{" not in tpl, tpl[tpl.index("{{") : tpl.index("{{") + 80]
    return tpl


def rows(log, tag):
    out = []
    for line in read(os.path.join(LOGS, log)).splitlines():
        if line.startswith(tag + " "):
            out.append([float(v) for v in line.split()[1:]])
    return out


def data():
    d = {}
    d["fall"] = [[float(l.split()[1]), float(l.split()[2])] for l in read(os.path.join(LOGS, "log_step02_physics.txt")).splitlines() if l.startswith("Z ")]
    d["bbox"] = {}
    for line in read(os.path.join(LOGS, "log_step03_scene.txt")).splitlines():
        m = re.match(r"BBOX (\S+) x (\S+)~(\S+)\s+y (\S+)~(\S+)\s+top z (\S+)", line)
        if m:
            p, *v = m.groups()
            d["bbox"][p] = dict(zip(("x0", "x1", "y0", "y1", "top"), map(float, v)))
    d["gofa"] = [r[:5] for r in rows("log_step04_gofa.txt", "G")]
    g = read(os.path.join(LOGS, "log_step04_gofa.txt"))
    d["gofaRoot"] = re.search(r"^ROOT (.*)$", g, re.M).group(1)
    d["gofaDof"] = re.search(r"^GOFA (.*)$", g, re.M).group(1)
    d["dofs"] = re.findall(r"^ROBOT (\w+) dof (\d+): (.*)$", read(os.path.join(LOGS, "log_step05_robots.txt")), re.M)
    d["joint"] = rows("log_step06_joints.txt", "J")
    e = rows("log_step07_ik.txt", "E")
    d["ik"] = [[r[0], r[1]] for r in e]
    d["ikxy"] = [{"x": r[4], "y": r[5]} for r in e if r[0] >= 1.0]
    d["rapid"] = rows("log_step08_rapid.txt", "P")
    cyc = json.load(open(os.path.join(HERE, "cycle.json"), encoding="utf-8"))["rows"]
    d["cycle"] = [[round(t - 8.0, 3), v, z] for t, v, z in cyc if 8.0 <= t <= 16.0]
    return d


if __name__ == "__main__":
    tpl = read(os.path.join(HERE, "template.html"))
    page = fill(tpl)
    d = data()
    # 표는 데이터로 채운다
    bbox_rows = "".join(
        f'<tr><td><code>{p}</code></td><td class="num">{b["x0"]:.3f} ~ {b["x1"]:.3f}</td><td class="num">{b["y0"]:.3f} ~ {b["y1"]:.3f}</td><td class="num">{b["top"]:.3f}</td></tr>'
        for p, b in d["bbox"].items())
    page = page.replace('<table id="bbox-table"><thead><tr><th>프림</th><th class="num">x 범위 [m]</th><th class="num">y 범위 [m]</th><th class="num">윗면 z [m]</th></tr></thead><tbody></tbody>',
                        '<table id="bbox-table"><thead><tr><th>프림</th><th class="num">x 범위 [m]</th><th class="num">y 범위 [m]</th><th class="num">윗면 z [m]</th></tr></thead><tbody>' + bbox_rows + "</tbody>")
    dof_rows = "".join(f'<tr><td>{n}</td><td class="num">{k}</td><td><code>{html.escape(v)}</code></td></tr>' for n, k, v in d["dofs"])
    page = page.replace('<tbody id="dof-table"></tbody>', '<tbody id="dof-table">' + dof_rows + "</tbody>")
    gofa_rows = (f'<tr><td>ROOT</td><td><code>{html.escape(d["gofaRoot"])}</code></td></tr>'
                 f'<tr><td>GOFA</td><td><code>{html.escape(d["gofaDof"])}</code></td></tr>')
    page = page.replace('<tbody id="gofa-table"></tbody>', '<tbody id="gofa-table">' + gofa_rows + "</tbody>")
    with open(os.path.join(HERE, "index.html"), "w", encoding="utf-8") as f:
        f.write(page)
    with open(os.path.join(HERE, "data.js"), "w", encoding="utf-8") as f:
        f.write("// 단계 스크립트 실행 로그에서 뽑은 그래프 수치 (build.py가 만든다)\nwindow.LEARN_DATA = " + json.dumps(d, ensure_ascii=False) + ";\n")
    print("index.html", len(page), "data.js keys", list(d), {k: len(v) for k, v in d.items() if isinstance(v, list)})
