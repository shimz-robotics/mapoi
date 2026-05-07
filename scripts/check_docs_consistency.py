#!/usr/bin/env python3
"""mapoi_interfaces の docs ↔ implementation 整合性 lint (#216)。

PR #215 (Close #207) で `mapoi_interfaces/msg/NavigationBackendStatus.msg` /
`mapoi_interfaces/README.md` / `mapoi_server/src/mapoi_nav_server.cpp` の 3
ファイル間に pointer ベースの cross-reference を貼った (custom bridge 向け
`backend_ready` 算出ガイダンス + `reason` 機微情報 redaction 指針)。一次情報を
msg コメントに集約して README / cpp は msg を pointer する設計だが、片方だけ
更新されると pointer が orphan 化し、custom bridge 実装者を誤誘導する。

CI で本 script を回し、以下を検知する:

1. README 節 presence: `mapoi_interfaces/CMakeLists.txt` の
   `rosidl_generate_interfaces(...)` に列挙された `.msg` / `.srv` 全件について、
   `mapoi_interfaces/README.md` に `### {Name}.{ext}` 節があるか。逆向きの orphan
   節 (README にあるが CMakeLists にない) も検出する。

2. Cross-reference 存在検証: 走査対象内の
   `mapoi_interfaces/(msg|srv)/X.(msg|srv)` 参照について、相手ファイルが実在するか
   確認する。

3. `reason` 文字列の機微情報 redaction static check: C++ ソース中の
   `publish_backend_status` 系関数本体から `reason = "..."` / `reason += "..."`
   の文字列リテラル成分のみ抽出し、絶対パス / IP リテラル / 機密語 / hostname
   らしき literal を含まないか確認する (パラメータ連結 `+ this->get_parameter(...)`
   等の dynamic 部分は対象外)。

設計方針:

- (1)(2) は CMakeLists の inventory ベースで自動展開され、新 msg / srv 追加時の
  README 節欠落も検知される。LocalizationBackendStatus (#209) など contract msg
  の追加にも追従する。
- (3) は `BACKEND_STATUS_FUNCTIONS` で対象関数名を持ち、bridge 追加時にここを
  更新するだけで済む。

使い方::
    python3 scripts/check_docs_consistency.py

Exit codes:
- 0: OK
- 1: 整合性違反を検出
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
INTERFACES_PKG = REPO_ROOT / 'mapoi_interfaces'
INTERFACES_README = INTERFACES_PKG / 'README.md'
INTERFACES_CMAKELISTS = INTERFACES_PKG / 'CMakeLists.txt'

# `reason` redaction static check の対象関数 (#216 (3))。
# bridge 増加時はここに追加する。関数名のみで file path 非依存にすることで、
# 関数を別 file に移しても follow できる。
BACKEND_STATUS_FUNCTIONS = (
    'publish_backend_status',
)

# `reason` 文字列リテラルに含まれてはいけない pattern。
# 各 pattern は `(label, regex)` で、検出時は label を fail メッセージに出す。
# false positive を避けるため、pattern は string literal 内のみに適用する
# (関数本体の生 source ではなく、抽出した string literal token のみ走査)。
FORBIDDEN_PATTERNS: tuple[tuple[str, re.Pattern[str]], ...] = (
    ('absolute path', re.compile(r'(?:^|[\s\'"`(])(/(?:home|etc|usr|var)/)')),
    ('IP literal', re.compile(r'\b(?:\d{1,3}\.){3}\d{1,3}\b')),
    ('credential keyword', re.compile(r'\b(?:password|secret|credential|api[_-]?key|token)\b', re.IGNORECASE)),
    ('internal hostname', re.compile(r'\b[a-z][a-z0-9-]*\.(?:local|lan|internal|corp)\b', re.IGNORECASE)),
)


def parse_cmake_inventory() -> dict[str, list[str]]:
    """`mapoi_interfaces/CMakeLists.txt` の `rosidl_generate_interfaces(...)` から
    msg / srv の inventory を抽出する。

    返り値: ``{'msg': [...], 'srv': [...]}`` (basename のみ、拡張子なし)。
    """
    text = INTERFACES_CMAKELISTS.read_text()
    m = re.search(r'rosidl_generate_interfaces\s*\([^)]*?\)', text, re.DOTALL)
    if not m:
        raise SystemExit(
            f'ERROR: rosidl_generate_interfaces() block が見つかりません: '
            f'{INTERFACES_CMAKELISTS.relative_to(REPO_ROOT)}'
        )
    block = m.group(0)
    inventory: dict[str, list[str]] = {'msg': [], 'srv': []}
    # "msg/Foo.msg" / "srv/Bar.srv" のみ抽出 (DEPENDENCIES 等は無視)。
    for kind, name in re.findall(r'"(msg|srv)/([A-Za-z0-9_]+)\.(?:msg|srv)"', block):
        inventory[kind].append(name)
    return inventory


def check_readme_section_presence(inventory: dict[str, list[str]]) -> list[str]:
    """README に各 msg/srv の `### {Name}.{ext}` 節があるか確認する。

    逆向き orphan (README にあるが CMakeLists にない) も検出する。
    """
    issues: list[str] = []
    if not INTERFACES_README.exists():
        return [f'{INTERFACES_README.relative_to(REPO_ROOT)} が存在しません']

    readme_text = INTERFACES_README.read_text()
    # `### Foo.msg` / `### Foo.srv` パターン
    found_sections: set[str] = set()
    for m in re.finditer(r'^###\s+([A-Za-z0-9_]+)\.(msg|srv)\s*$', readme_text, re.MULTILINE):
        found_sections.add(f'{m.group(1)}.{m.group(2)}')

    expected: set[str] = set()
    for kind, names in inventory.items():
        for n in names:
            expected.add(f'{n}.{kind}')

    missing = expected - found_sections
    orphan = found_sections - expected

    for name in sorted(missing):
        issues.append(
            f'README 節欠落: `### {name}` が '
            f'{INTERFACES_README.relative_to(REPO_ROOT)} にない '
            f'(CMakeLists には登録済み)'
        )
    for name in sorted(orphan):
        issues.append(
            f'README orphan 節: `### {name}` が '
            f'{INTERFACES_README.relative_to(REPO_ROOT)} にあるが '
            f'CMakeLists.txt に対応 .msg/.srv が無い'
        )
    return issues


def collect_scan_targets() -> list[Path]:
    """cross-reference 検証で走査するファイル一覧を返す。

    対象: mapoi_interfaces 配下の `.msg` / `.srv` / README、および BACKEND_STATUS_FUNCTIONS
    のいずれかを含む `.cpp` / `.hpp`。
    """
    targets: list[Path] = []
    targets.append(INTERFACES_README)
    targets.extend(sorted((INTERFACES_PKG / 'msg').glob('*.msg')))
    targets.extend(sorted((INTERFACES_PKG / 'srv').glob('*.srv')))
    for ext in ('cpp', 'hpp'):
        for p in REPO_ROOT.rglob(f'*.{ext}'):
            # build / install / log 配下は除外
            rel = p.relative_to(REPO_ROOT).parts
            if rel and rel[0] in ('build', 'install', 'log', '.git'):
                continue
            text = p.read_text(errors='replace')
            if any(fn in text for fn in BACKEND_STATUS_FUNCTIONS):
                targets.append(p)
    return targets


def check_cross_references(targets: list[Path]) -> list[str]:
    """走査対象内の `mapoi_interfaces/(msg|srv)/X.(msg|srv)` 参照が実在するか確認する。"""
    issues: list[str] = []
    pattern = re.compile(r'mapoi_interfaces/(msg|srv)/([A-Za-z0-9_]+)\.(msg|srv)')
    for path in targets:
        text = path.read_text(errors='replace')
        for lineno, line in enumerate(text.splitlines(), start=1):
            for m in pattern.finditer(line):
                kind, name, ext = m.group(1), m.group(2), m.group(3)
                if kind != ext:
                    issues.append(
                        f'{path.relative_to(REPO_ROOT)}:{lineno}: '
                        f'kind 不一致 ({kind}/{name}.{ext})'
                    )
                    continue
                target = INTERFACES_PKG / kind / f'{name}.{ext}'
                if not target.exists():
                    issues.append(
                        f'{path.relative_to(REPO_ROOT)}:{lineno}: '
                        f'参照先不在 `{m.group(0)}` -> {target.relative_to(REPO_ROOT)}'
                    )
    return issues


def extract_function_body(text: str, func_name: str) -> tuple[int, str] | None:
    """C++ ソースから ``func_name`` の関数本体を抽出する。

    ``Foo::func_name`` / ``ReturnType func_name`` のいずれの定義形式にも対応する
    ため、関数名 + 開き ``{`` の組を起点に対応 brace を数える。

    返り値: ``(行番号 (開始 brace), 本体文字列)`` または None。
    """
    # 関数定義の「名前」部分を見つける (引数 list を経て `)` の後の `{` まで)。
    # 簡易: `func_name\s*\(` を探し、対応する `)` を見つけ、その後の最初の `{` から
    # 対応 brace まで抽出する。
    for name_match in re.finditer(rf'\b{re.escape(func_name)}\s*\(', text):
        start_paren = name_match.end() - 1
        depth = 1
        i = start_paren + 1
        while i < len(text) and depth > 0:
            if text[i] == '(':
                depth += 1
            elif text[i] == ')':
                depth -= 1
            i += 1
        if depth != 0:
            continue
        # i は `)` の次。次の `{` を探す。
        # ただし宣言だけ (i.e. 後続が `;`) ならスキップ。
        rest = text[i:].lstrip()
        if not rest or rest[0] != '{':
            continue
        body_start = text.index('{', i)
        depth = 1
        j = body_start + 1
        while j < len(text) and depth > 0:
            if text[j] == '{':
                depth += 1
            elif text[j] == '}':
                depth -= 1
            j += 1
        if depth != 0:
            continue
        body_lineno = text.count('\n', 0, body_start) + 1
        return (body_lineno, text[body_start + 1:j - 1])
    return None


def extract_reason_string_literals(body: str) -> list[tuple[int, str]]:
    """関数本体から ``reason = "..."`` / ``reason += "..."`` / ``reason +=`` 連鎖の
    string literal 成分を抽出する。

    パラメータ連結 (``+ this->get_parameter(...).as_string()``) の動的部分は
    対象外で、明示的な文字列リテラルのみを返す。
    """
    literals: list[tuple[int, str]] = []
    # `(.+?\.)?reason\s*(=|\+=)` で始まる statement を捕まえ、その statement 全体
    # (次の `;` まで) を取得して string literal を切り出す。
    for stmt_match in re.finditer(r'(?:(?:[A-Za-z_][A-Za-z0-9_]*\.)?reason)\s*(?:=|\+=)', body):
        start = stmt_match.start()
        # statement 終端 (`;`) を探す。string literal 内の `;` を避けるため簡易 scanner。
        i = stmt_match.end()
        in_string = False
        escape = False
        end = -1
        while i < len(body):
            ch = body[i]
            if in_string:
                if escape:
                    escape = False
                elif ch == '\\':
                    escape = True
                elif ch == '"':
                    in_string = False
            else:
                if ch == '"':
                    in_string = True
                elif ch == ';':
                    end = i
                    break
            i += 1
        if end < 0:
            continue
        statement = body[start:end]
        # statement 内の `"..."` literal を抽出 (escape 対応)。
        for s_match in re.finditer(r'"((?:[^"\\]|\\.)*)"', statement):
            lineno_in_body = body.count('\n', 0, start + s_match.start()) + 1
            literals.append((lineno_in_body, s_match.group(1)))
    return literals


def check_reason_redaction(targets: list[Path]) -> list[str]:
    """C++ ソースの BACKEND_STATUS_FUNCTIONS 関数本体から `reason` 文字列リテラル
    を抽出し、FORBIDDEN_PATTERNS のいずれかにマッチしたら fail。"""
    issues: list[str] = []
    cpp_targets = [p for p in targets if p.suffix in ('.cpp', '.hpp')]
    for path in cpp_targets:
        text = path.read_text(errors='replace')
        for func_name in BACKEND_STATUS_FUNCTIONS:
            extracted = extract_function_body(text, func_name)
            if extracted is None:
                continue
            body_lineno, body = extracted
            literals = extract_reason_string_literals(body)
            for lineno_in_body, literal in literals:
                abs_lineno = body_lineno + lineno_in_body - 1
                for label, pattern in FORBIDDEN_PATTERNS:
                    m = pattern.search(literal)
                    if m:
                        issues.append(
                            f'{path.relative_to(REPO_ROOT)}:{abs_lineno}: '
                            f'`{func_name}` の reason に {label} 検出 '
                            f'(matched: {m.group(0)!r}, literal: {literal!r})'
                        )
    return issues


def main() -> int:
    if not INTERFACES_PKG.is_dir():
        print(f'ERROR: {INTERFACES_PKG} が存在しません', file=sys.stderr)
        return 1

    inventory = parse_cmake_inventory()
    targets = collect_scan_targets()

    all_issues: list[tuple[str, list[str]]] = [
        ('README section presence', check_readme_section_presence(inventory)),
        ('cross-reference integrity', check_cross_references(targets)),
        ('reason redaction (sensitive info)', check_reason_redaction(targets)),
    ]

    failed = False
    for label, issues in all_issues:
        if issues:
            failed = True
            print(f'[FAIL] {label}', file=sys.stderr)
            for i in issues:
                print(f'  - {i}', file=sys.stderr)
        else:
            print(f'[OK]   {label}')

    if failed:
        return 1

    msg_count = len(inventory['msg'])
    srv_count = len(inventory['srv'])
    cpp_count = sum(1 for p in targets if p.suffix in ('.cpp', '.hpp'))
    print(
        f'OK: mapoi_interfaces docs/implementation 整合性 OK '
        f'(msg={msg_count}, srv={srv_count}, scanned cpp={cpp_count})'
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
