#!/usr/bin/env bash
# [工具] 文档自检：检查 Markdown 文档的结构规范与交叉引用是否自洽。
#
# 用法（在仓库根目录，或任意目录——脚本会自己定位仓库根）：
#   bash scripts/check_docs.sh
#
# 检查 5 项：
#   1) 代码栅栏（``` ）是否成对             —— 少一个就把后面全渲染坏
#   2) 代码块里有没有尖括号占位符           —— 复制粘贴到 bash 会被当成重定向而报错
#   3) README 修订记录的版本号是否升序       —— 插入新版本时最容易犯错的地方
#   4) README / notes 里的 `log.md §X.Y` 引用是否都存在
#   5) README / notes 里引用的仓库文件是否真实存在
#
# 退出码：0 = 全部通过；1 = 有失败项。
set -eo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

python3 - <<'PY'
import os
import re
import sys

GREEN, YELLOW, RED, BOLD, RESET = '\033[32m', '\033[33m', '\033[31m', '\033[1m', '\033[0m'
FAILED = 0
WARNED = 0


def head(title):
    print(f'\n{BOLD}{title}{RESET}')


def ok(msg):
    print(f'  {GREEN}[ OK ]{RESET} {msg}')


def warn(msg):
    global WARNED
    WARNED += 1
    print(f'  {YELLOW}[警告]{RESET} {msg}')


def bad(msg):
    global FAILED
    FAILED += 1
    print(f'  {RED}[失败]{RESET} {msg}')


def read(path):
    with open(path, encoding='utf-8') as f:
        return f.read()


DOCS = ['README.md', 'docs/notes.md', 'docs/log.md']
missing = [d for d in DOCS if not os.path.exists(d)]
if missing:
    bad(f'缺少文档：{missing}')
    sys.exit(1)

# 代码块里的文本（顶层 ``` 与引用块里 > ``` 都算），供第 2 项使用
FENCE = re.compile(r'^\s*>?\s*```')


def code_blocks(text):
    """返回所有围栏代码块的内容（按行列表的列表）。"""
    blocks, cur, inside = [], [], False
    for line in text.split('\n'):
        if FENCE.match(line):
            if inside:
                blocks.append(cur)
            cur, inside = [], not inside
            continue
        if inside:
            cur.append(line)
    return blocks


# ---------- 1) 代码栅栏成对 ----------
head('[1/5] 代码栅栏是否成对')
for d in DOCS:
    n = sum(1 for line in read(d).split('\n') if FENCE.match(line))
    if n % 2 == 0:
        ok(f'{d}：{n} 个围栏（成对）')
    else:
        bad(f'{d}：{n} 个围栏 —— 奇数，有代码块没闭合')

# ---------- 2) 代码块内没有尖括号占位符 ----------
head('[2/5] 代码块里有没有尖括号占位符（会破坏复制粘贴）')
if FAILED:
    warn('第 1 项已经失败（围栏不配平）—— 下面的代码块切分不可靠，先修围栏再看本项')
PLACEHOLDER = re.compile(r'<[^<>\n]{1,40}>')
for d in ['README.md', 'docs/notes.md']:
    hits = []
    for block in code_blocks(read(d)):
        for line in block:
            if '#include' in line:          # C++ 头文件里的 <vector> 之类不算占位符
                continue
            for m in PLACEHOLDER.finditer(line):
                hits.append((line.strip(), m.group(0)))
    if not hits:
        ok(f'{d}：代码块内无尖括号占位符')
    else:
        for line, token in hits[:5]:
            bad(f'{d}：`{token}` —— 出现在 `{line[:60]}`')
        if len(hits) > 5:
            bad(f'{d}：另有 {len(hits) - 5} 处')

# ---------- 3) 修订记录版本号升序 ----------
head('[3/5] README 修订记录的版本号顺序')
readme = read('README.md')
versions = [re.match(r'^\| \*{0,2}(v[\d.]+)\*{0,2} \|', line).group(1)
            for line in readme.split('\n')
            if re.match(r'^\| \*{0,2}v[\d.]+\*{0,2} \|', line)]
key = lambda v: tuple(int(x) for x in v.lstrip('v').split('.'))
if not versions:
    bad('没有解析到任何版本行（修订记录表结构变了？）')
elif versions == sorted(versions, key=key):
    ok(f'共 {len(versions)} 个版本，按 {versions[0]} → {versions[-1]} 升序')
else:
    bad(f'版本号乱序：{versions}')

# ---------- 4) § 交叉引用是否有效 ----------
head('[4/5] README / notes 里的 `log.md §X.Y` 引用')
log = read('docs/log.md')
chapters = set(re.findall(r'^## (\d+)\.', log, re.M))
sections = set(re.findall(r'^### (\d+\.\d+)', log, re.M))
print(f'         log.md 现有 {len(chapters)} 章 / {len(sections)} 小节')
for d in ['README.md', 'docs/notes.md']:
    refs = {m.group(1) for m in re.finditer(r'§(\d+(?:\.\d+)?)', read(d))}
    dangling = sorted(
        r for r in refs
        if (r not in sections if '.' in r else r not in chapters)
    )
    if dangling:
        bad(f'{d}：{len(dangling)} 个失效引用 {dangling}')
    else:
        ok(f'{d}：{len(refs)} 个 §引用全部有效')

# ---------- 5) 路径引用是否存在 ----------
head('[5/5] README / notes 里引用的仓库文件')
PATH_RE = re.compile(
    r'`((?:docs|models|scripts|data|01_detector|02_tracker|03_visualization|04_hik|rm_interfaces)/[^`\s]*?)`'
)
for d in ['README.md', 'docs/notes.md']:
    checked = 0
    broken = set()
    for line in read(d).split('\n'):
        if re.match(r'^\| \*{0,2}v[\d.]+\*{0,2} \|', line):
            continue                        # 修订记录里会提到已删除的旧文件，跳过
        for path in PATH_RE.findall(line):
            if any(c in path for c in '*{}<>') or path.endswith('/'):
                continue                    # 通配/占位符/目录，跳过
            checked += 1
            if not os.path.exists(path):
                broken.add(path)
    if broken:
        shown = sorted(broken)[:10]
        bad(f'{d}：{len(broken)} 个不存在的路径 —— ' + '、'.join(f'`{p}`' for p in shown)
            + ('…' if len(broken) > len(shown) else ''))
    else:
        ok(f'{d}：{checked} 个路径引用全部存在')

# ---------- 结论 ----------
head('== 文档自检结论 ==')
if FAILED:
    print(f'  {RED}{FAILED} 项失败{RESET}，{WARNED} 项警告 —— 请先修掉上面的失败项。')
    sys.exit(1)
print(f'  {GREEN}全部通过{RESET}（{WARNED} 项警告）—— 文档结构规范、交叉引用自洽。')
PY
