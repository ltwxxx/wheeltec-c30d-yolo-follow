"""Offline HTML report with truth-based curves and explicit fidelity boundaries."""
import csv
from html import escape
import math


def curve(rows, keys, labels, unit):
    colors = ('#0071e3', '#777777')
    values = [float(row[key]) for row in rows for key in keys]
    low, high = min(values), max(values)
    pad = max(.05, (high-low)*.10)
    low, high = low-pad, high+pad
    duration = max(float(rows[-1]['time_s']), 1e-6)
    pieces = [f'<svg viewBox="0 0 720 230" role="img" '
              f'aria-label="{escape(" / ".join(labels))}">',
              '<path d="M52 12 V192 H704" fill="none" stroke="#ccc"/>']
    for key, color in zip(keys, colors):
        selected = rows[::max(1, len(rows)//600)]
        if selected[-1] is not rows[-1]:
            selected.append(rows[-1])
        points = ' '.join(f'{52+float(r["time_s"])/duration*645:.2f},'
                          f'{192-(float(r[key])-low)/(high-low)*175:.2f}' for r in selected)
        pieces.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2"/>')
    pieces += [f'<text x="2" y="20">{high:.2f}</text>',
               f'<text x="2" y="192">{low:.2f}</text>',
               '<text x="52" y="219">0 s</text>',
               f'<text x="640" y="219">{duration:.1f} s</text>', '</svg>']
    legend = ' · '.join(f'<span style="color:{color}">{escape(label)}</span>'
                        for label, color in zip(labels, colors))
    return f'<div class="plot"><div>{legend} / {escape(unit)}</div>{"".join(pieces)}</div>'


def wilson(successes, count):
    z = 1.96
    p = successes/count
    center = (p+z*z/(2*count))/(1+z*z/count)
    half = z*math.sqrt(p*(1-p)/count+z*z/(4*count*count))/(1+z*z/count)
    return center-half, center+half


def write_report(output, manifest, summaries):
    count = len(summaries)
    successes = sum(s['success'] for s in summaries)
    low, high = wilson(successes, count)
    rows_html, details = [], []
    for summary in summaries:
        case = summary['case_id']
        with (output/case/'telemetry.csv').open(encoding='utf-8', newline='') as stream:
            rows = list(csv.DictReader(stream))
        for row in rows:
            row['bearing_deg'] = str(math.degrees(float(row['true_bearing_rad'])))
        reasons = ', '.join(summary['failure_reasons']) or '—'
        status = '通过' if summary['success'] else '未通过'
        rows_html.append(f'<tr><td><a href="#{escape(case)}">{escape(case)}</a></td>'
                         f'<td>{status}</td><td>{summary["mean_band_error_m"]:.3f}</td>'
                         f'<td>{summary["p95_band_error_m"]:.3f}</td>'
                         f'<td>{summary["collision_events"]}</td>'
                         f'<td>{escape(reasons)}</td></tr>')
        charts = curve(rows, ['true_gap_m'], ['真实间距'], 'm')
        charts += curve(rows, ['bearing_deg'], ['真实方位误差'], '°')
        charts += curve(rows, ['left_rad_s', 'right_rad_s'], ['左轮指令', '右轮指令'], 'rad/s')
        details.append(f'<details id="{escape(case)}"><summary>{escape(case)} · {status}</summary>'
                       f'<p>失败原因：{escape(reasons)}。完整指标见 '
                       f'<a href="{escape(case)}/summary.json">summary.json</a>；'
                       f'<a href="{escape(case)}/telemetry.csv">逐时刻CSV</a>。</p>'
                       f'<div class="charts">{charts}</div></details>')
    mode = ('仿真真实位置（不评感知）' if manifest['observation'] == 'truth'
            else '渲染RGB + 橙色像素识别（未接入YOLO）')
    target = manifest['controller']['target_m']
    deadband = manifest['controller']['distance_deadband_m']
    html = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>跟随仿真实验报告</title>
<style>
:root{{font-family:"PingFang SC",system-ui,sans-serif;color:#1d1d1f;background:#f5f5f7}}
body{{max-width:1200px;margin:auto;padding:48px 24px 80px;line-height:1.7}}
h1{{font-size:42px;line-height:1.2}}h2{{margin-top:38px}}a{{color:#0071e3}}
.eyebrow{{color:#6e6e73;font-size:14px;letter-spacing:.12em}}.lead{{font-size:19px;color:#6e6e73}}
.stats{{display:grid;grid-template-columns:repeat(3,1fr);gap:16px;margin:28px 0}}
.stats article,section,details{{background:white;padding:24px;border-radius:18px;margin:14px 0}}
.stats article{{margin:0}}.metric{{font-size:36px;color:#0071e3;font-weight:600}}.small{{font-size:14px;color:#6e6e73}}
table{{width:100%;border-collapse:collapse;text-align:left}}th,td{{border-bottom:1px solid #e5e5ea;padding:13px 12px}}.table{{overflow:auto}}
details summary{{font-size:19px;cursor:pointer;font-weight:600}}.charts{{display:grid;grid-template-columns:1fr 1fr;gap:16px}}
.plot{{min-width:0}}svg{{width:100%}}svg text{{font-size:12px;fill:#777}}code{{background:#eee;padding:3px 6px;border-radius:5px}}
@media(max-width:700px){{.stats,.charts{{grid-template-columns:1fr}}h1{{font-size:32px}}body{{padding:24px 14px}}}}
</style><div class="eyebrow">FOLLOW EXPERIMENT / CONTROL BASELINE</div>
<h1>跟随仿真实验报告</h1><p class="lead">可重复的控制基线 · 与实车差距仍待配对测量</p>
<p class="small">{escape(manifest['created_at'])} · MuJoCo {escape(manifest['runtime']['mujoco'])} · seed {manifest['seed']}</p>
<div class="stats"><article><div class="metric">{successes} / {count}</div>场景通过数</article>
<article><div class="metric">{successes/count:.1%}</div>本场景集通过率<br><span class="small">Wilson描述区间 {low:.1%}–{high:.1%}；非真实环境总体成功率</span></article>
<article><div class="metric">待测</div>Sim-to-Real gap<br><span class="small">尚无相同协议的配对真机数据</span></article></div>
<section><h2>本次到底验证了什么</h2><p>输入：{mode}。控制：教学P控制器，观测缺失时双轮零指令；与实车控制器尚未对齐。</p>
<p>评分间距：range_front测距点的XY投影，到人物胶囊平面轮廓的欧氏距离；目标距离带
{target-deadband:.2f}–{target+deadband:.2f} m。物理接触距离另列记录，不与上述间距混用。</p>
<p>通过条件：完成整段轨迹、没有车与人物的物理接触、观测缺失时零轮速指令，且最后
{manifest['evaluation']['hold_s']:.1f}秒持续处于目标距离带与
{manifest['evaluation']['bearing_tolerance_deg']:.0f}°方位容差内。移动阶段另看全程误差与在带比例。</p>
<p>模型边界：普通圆柱轮、未标定的摩擦与执行器；人物按给定轨迹运动；未模拟真实步态、完整二维雷达与车端推理延迟。人物暂失场景采用观测屏蔽，非物理遮挡。</p></section>
<section><h2>场景成绩</h2><div class="table"><table><thead><tr><th>场景</th><th>结果</th><th>平均带外误差/m</th><th>P95/m</th><th>接触事件</th><th>失败原因</th></tr></thead>
<tbody>{''.join(rows_html)}</tbody></table></div><p class="small">带外误差 = max(下界−间距, 0, 间距−上界)，含起步阶段；接触事件按物理子步统计接触从无到有的次数。</p></section>
<section><h2>怎样判断模型足够真实</h2><p>先定用途和误差预算，再做独立验证。先用部分实车数据标定参数，留出其他速度、地面和人物轨迹检验；仿真任务通过率不代表模型真实性。</p>
<div class="table"><table><tr><th>层次</th><th>配对测量</th><th>当前状态</th></tr>
<tr><td>几何</td><td>车身外廓、轮径轮距、相机与雷达外参</td><td>教学模型；实物核对待完成</td></tr>
<tr><td>动力学</td><td>同指令下的v/ω曲线、起步延迟、上升时间、制动距离</td><td>未标定</td></tr>
<tr><td>感知与时序</td><td>相同条件下检测漏检/误检、方位与测距误差、帧龄与端到端延迟P50/P95</td><td>YOLO链路未接入；真值模式不评感知</td></tr>
<tr><td>闭环任务</td><td>相同控制器、任务、初始条件分组，比较成功率百分点差、误差分布和人工接管</td><td>缺少配对真机数据</td></tr></table></div></section>
<h2>逐场景曲线</h2>{''.join(details)}
<section><h2>复现与材料</h2><p>保留整个输出目录。manifest记录场景、参数、运行环境和代码校验；model.xml保留原模型快照。重跑时校验代码、模型与MuJoCo版本。</p>
<p><code>python sim/run_experiments.py --replay PATH/manifest.json --case static-01</code></p>
<p><a href="manifest.json">实验清单</a> · <a href="summary.json">完整指标JSON</a> · <a href="summary.csv">汇总CSV</a></p>
<p class="small">同版本、同平台、同输入用于数值复现；跨硬件不承诺逐位一致。固定场景集上的Wilson区间仅作描述，不能外推实际部署表现。</p></section>
<script>function reveal(){{let el=document.getElementById(location.hash.slice(1));if(el&&el.tagName==='DETAILS')el.open=true;}}addEventListener('hashchange',reveal);reveal();</script></html>'''
    (output/'report.html').write_text(html, encoding='utf-8')
