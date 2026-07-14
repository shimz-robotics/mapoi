#!/usr/bin/env python3
"""bloom-release の前提条件が壊れていないかを PR 時点で検証する (#454)。

#395 (PR #453) で整えたリリース基盤 (per-package ``CHANGELOG.rst`` + ``LICENSE``
同梱) は、通常のビルド・テストでは健全性が確認できず、次の bloom-release 実行時に
初めて破綻が露見する。本 script はその前提条件を軽量 lint として検証する。

対象は ``package.xml`` を持つ全ディレクトリ (= bloom がパッケージ単位で source を
切り出す単位)。以下を検査する:

1. **CHANGELOG.rst の parse 可能性** — bloom が使う
   ``catkin_pkg.changelog.get_changelog_from_path`` で読める (None にならない)。
2. **版見出しの underline 崩れ検知** — docutils の構造警告 (WARNING 以上) を検出。
   版・日付の桁を増やして underline を伸ばし忘れる典型ミスを拾う。catkin_pkg 自体は
   短い underline を寛容に通してしまうため、docutils の doctree を直接検査する。
3. **版見出しの消失検知** — テキスト上の ``X.Y.Z (YYYY-MM-DD)`` 見出しが catkin_pkg
   に版として認識されているか突き合わせる (underline 完全欠落で見出しが段落に化けた
   ケースを検出)。
4. **Forthcoming 節の存在** — bloom が次リリースを注入する ``Forthcoming`` 節が
   section として健在か (節の削除・underline 崩れを検出)。
5. **LICENSE / CHANGELOG.rst の同梱** — ``package.xml`` の隣に両ファイルが存在するか
   (新パッケージ追加時の同梱漏れ防止。漏れると deb にライセンス・changelog が入らない)。

``catkin_pkg`` / ``docutils`` は pure Python なので ROS 環境不要の軽量 job で実行可能。

**スコープ**: 本 script はリリース基盤のドリフト防止を目的とした軽量 lint で、
changelog 本文の内容妥当性 (版番号の昇順・日付の妥当性・エントリ文言など) や
LICENSE 本文の正しさは検査しない。

使い方::
    python3 scripts/check_release_packaging.py

Exit codes:
- 0: OK
- 1: 前提条件違反を検出 / 解析環境不備
"""
from __future__ import annotations

import contextlib
import io
import re
import sys
from pathlib import Path

import docutils.core
import docutils.nodes
from catkin_pkg.changelog import CHANGELOG_FILENAME, get_changelog_from_path

REPO_ROOT = Path(__file__).resolve().parent.parent

# bloom がパッケージ単位で source を切り出すため各パッケージに同梱が必須なファイル。
REQUIRED_SIBLINGS = (CHANGELOG_FILENAME, 'LICENSE')

# package.xml 探索から除外するディレクトリ (ビルド生成物・依存物)。
IGNORE_DIRS = {'node_modules', 'build', 'install', 'log', '.git'}

# 版見出しのテキスト表現 (行全体が ``X.Y.Z (YYYY-MM-DD)``)。
VERSION_HEADING_RE = re.compile(r'^(\d+\.\d+\.\d+) \(\d{4}-\d{2}-\d{2}\)$', re.MULTILINE)

FORTHCOMING_LABEL = 'Forthcoming'

# doctitle/subtitle への昇格変換を切って全見出しを section として扱い、警告は buffer へ
# 逃がして CI ログを汚さない (node は doctree から拾う)。
_DOCUTILS_OVERRIDES = {
    'report_level': 1,
    'halt_level': 5,
    'doctitle_xform': False,
    'sectsubtitle_xform': False,
}


def _find_package_dirs() -> list[Path]:
    dirs: list[Path] = []
    for pkg_xml in REPO_ROOT.rglob('package.xml'):
        rel_parts = pkg_xml.relative_to(REPO_ROOT).parts
        if any(part in IGNORE_DIRS for part in rel_parts):
            continue
        dirs.append(pkg_xml.parent)
    return sorted(dirs)


def _section_titles_and_warnings(rst: str) -> tuple[list[str], list[str]]:
    """RST を docutils で解析し、section title 一覧と WARNING 以上の構造警告を返す。"""
    doctree = docutils.core.publish_doctree(
        rst,
        settings_overrides={**_DOCUTILS_OVERRIDES, 'warning_stream': io.StringIO()},
    )
    titles: list[str] = []
    for section in doctree.findall(docutils.nodes.section):
        for title in section.findall(docutils.nodes.title):
            titles.append(title.astext())
            break
    warnings = [
        node.astext().splitlines()[0]
        for node in doctree.findall(docutils.nodes.system_message)
        if node['level'] >= 2
    ]
    return titles, warnings


def _check_changelog(pkg_dir: Path) -> list[str]:
    issues: list[str] = []
    rel = pkg_dir.relative_to(REPO_ROOT)
    path = pkg_dir / CHANGELOG_FILENAME
    if not path.exists():
        # 同梱チェック側で報告するのでここでは skip。
        return issues

    rst = path.read_text(encoding='utf-8')

    # 2 + 4: 構造警告と Forthcoming 節の健在性を docutils の doctree から検査。
    titles, warnings = _section_titles_and_warnings(rst)
    for warning in warnings:
        issues.append(f'{rel}/{CHANGELOG_FILENAME}: RST 構造警告 (見出し underline 崩れ等): {warning}')
    if FORTHCOMING_LABEL not in titles:
        issues.append(
            f'{rel}/{CHANGELOG_FILENAME}: "{FORTHCOMING_LABEL}" 節が section として存在しない '
            f'(節の削除、または underline 崩れで段落化。bloom が次版を注入できなくなる)'
        )

    # 1: bloom が使う reader で parse 可能か。内部の docutils が default 設定で
    # stderr に警告を吐く (崩れは既に上で検出済み) ため、二重出力を避けて捨てる。
    with contextlib.redirect_stderr(io.StringIO()):
        changelog = get_changelog_from_path(str(path))
    if changelog is None:
        issues.append(
            f'{rel}/{CHANGELOG_FILENAME}: catkin_pkg.get_changelog_from_path が None を返した (読めない)'
        )
        return issues

    # 3: テキスト上の版見出しが catkin_pkg に版として認識されているか突き合わせ。
    parsed_versions = {version for version, _date, _content in changelog.foreach_version()}
    for match in VERSION_HEADING_RE.finditer(rst):
        version = match.group(1)
        if version not in parsed_versions:
            issues.append(
                f'{rel}/{CHANGELOG_FILENAME}: 版見出し "{match.group(0)}" が catkin_pkg で版として '
                f'認識されていない (underline 欠落・崩れで見出しが段落化した可能性)'
            )

    return issues


def _check_siblings(pkg_dir: Path) -> list[str]:
    rel = pkg_dir.relative_to(REPO_ROOT)
    return [
        f'{rel}/package.xml: 隣に "{name}" が無い (bloom の deb にライセンス/changelog が同梱されない)'
        for name in REQUIRED_SIBLINGS
        if not (pkg_dir / name).exists()
    ]


def main() -> int:
    pkg_dirs = _find_package_dirs()
    if not pkg_dirs:
        print('ERROR: package.xml を持つディレクトリが見つからない', file=sys.stderr)
        return 1

    issues: list[str] = []
    for pkg_dir in pkg_dirs:
        issues.extend(_check_siblings(pkg_dir))
        issues.extend(_check_changelog(pkg_dir))

    if issues:
        print('release packaging check failed:', file=sys.stderr)
        for issue in issues:
            print(f'  - {issue}', file=sys.stderr)
        return 1

    print(f'OK: {len(pkg_dirs)} package(s) の CHANGELOG.rst / LICENSE 同梱は健全。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
