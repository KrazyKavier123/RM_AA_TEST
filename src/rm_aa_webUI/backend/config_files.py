"""Edit existing config scalars by source span, retaining comments and OpenCV tags."""
import ast
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

import yaml


def revision(text):
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def scalar_type(value):
    return {bool: 'bool', int: 'int', float: 'float', str: 'str', type(None): 'null'}[type(value)]


def fields_for(text, launch=False):
    fields = []

    def add(key, label, value, start, end):
        if type(value) not in (bool, int, float, str, type(None)):
            return
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError(f'{label}: 不支持 NaN/Inf 参数')
        fields.append(dict(key=key, label=label, value=value, type=scalar_type(value), start=start, end=end))

    if launch:
        tree = ast.parse(text)
        lines = text.splitlines(keepends=True)

        def offset(line, column):
            # Python AST columns are UTF-8 byte offsets, including on Chinese comment lines.
            return sum(map(len, lines[:line-1])) + len(lines[line-1].encode('utf-8')[:column].decode('utf-8'))

        def literal(node, key, label):
            if isinstance(node, (ast.Constant, ast.UnaryOp)):
                add(key, label, ast.literal_eval(node), offset(node.lineno, node.col_offset),
                    offset(node.end_lineno, node.end_col_offset))

        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'DeclareLaunchArgument':
                if node.args and isinstance(node.args[0], ast.Constant):
                    name = str(node.args[0].value)
                    for kw in node.keywords:
                        if kw.arg == 'default_value':
                            literal(kw.value, f'argument:{node.lineno}:{name}', f'启动参数 / {name}')
            elif isinstance(node, ast.Dict):
                for key, value in zip(node.keys, node.values):
                    if isinstance(key, ast.Constant) and isinstance(key.value, str):
                        literal(value, f'dict:{value.lineno}:{key.value}', f'第 {value.lineno} 行 / {key.value}')
    else:
        # Same-length replacement preserves node offsets in OpenCV's nonstandard directive.
        parsed = text.replace('%YAML:1.0', '%YAML 1.1', 1) if text.startswith('%YAML:1.0') else text
        tree = yaml.compose(parsed, Loader=yaml.SafeLoader)
        if not isinstance(tree, yaml.MappingNode):
            raise ValueError('配置文件顶层必须是映射')

        def walk(node, path, ancestors):
            if id(node) in ancestors:
                raise ValueError('不支持循环 YAML 引用')
            if isinstance(node, yaml.MappingNode):
                keys = [key.value for key, _ in node.value]
                if len(keys) != len(set(keys)):
                    raise ValueError('配置文件存在重复键')
                for key, value in node.value:
                    walk(value, path + [key.value], ancestors | {id(node)})
            elif isinstance(node, yaml.SequenceNode):
                for index, value in enumerate(node.value):
                    walk(value, path + [str(index)], ancestors | {id(node)})
            else:
                # Do not construct !!opencv-matrix objects: only their scalar children.
                value = yaml.SafeLoader('').construct_object(node, deep=True)
                add('/' + '/'.join(p.replace('~', '~0').replace('/', '~1') for p in path),
                    ' / '.join(path), value, node.start_mark.index, node.end_mark.index)
        walk(tree, [], set())
    return fields


def edit_text(text, changes, launch=False):
    fields = {field['key']: field for field in fields_for(text, launch)}
    replacements = {}
    for key, value in changes.items():
        field = fields[key]
        kind = field['type']
        if kind == 'float' and type(value) in (int, float):
            value = float(value)
        if scalar_type(value) != kind:
            raise ValueError(f"{field['label']}: 参数类型必须保持为 {kind}")
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError('参数必须是有限数值')
        if launch:
            replacement = repr(value)
        elif isinstance(value, str):
            replacement = json.dumps(value, ensure_ascii=False)
        else:
            replacement = {True: 'true', False: 'false'}.get(value) if type(value) is bool else json.dumps(value)
        span = (field['start'], field['end'])
        if span in replacements and replacements[span] != replacement:
            raise ValueError('同一个 YAML 引用不能同时设置不同值')
        replacements[span] = replacement
    for (start, end), value in sorted(replacements.items(), reverse=True):
        text = text[:start] + value + text[end:]
    fields_for(text, launch)
    return text


class ConfigFiles:
    def __init__(self, workspace):
        self.root = Path(workspace).resolve()

    def paths(self):
        paths = []
        for directory in (self.root / 'src', self.root / 'training' / 'config'):
            for path in directory.rglob('*'):
                if not path.is_file() or not path.resolve().is_relative_to(self.root):
                    continue
                if ('config' in path.parts and path.name.endswith(('.yaml', '.yml', '.yaml.example'))) or (
                    'launch' in path.parts and path.name.endswith('.launch.py')):
                    paths.append(path.relative_to(self.root).as_posix())
        return sorted(paths)

    def resolve(self, name):
        if name not in self.paths():
            raise ValueError('请选择工程中的 config YAML 或 launch 文件')
        return self.root / name

    def describe(self, name, text=None):
        path = self.resolve(name)
        text = path.read_bytes().decode('utf-8-sig') if text is None else text
        return dict(path=name, revision=revision(text), content=text,
                    fields=fields_for(text, name.endswith('.py')))

    def save(self, name, expected_revision, content):
        path = self.resolve(name)
        current = path.read_bytes().decode('utf-8-sig')
        if revision(current) != expected_revision:
            raise FileExistsError('文件已在其他位置修改，请重新加载后再保存')
        fields_for(content, name.endswith('.py'))
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', newline='', dir=path.parent, delete=False) as file:
            file.write(content)
            temporary = Path(file.name)
        try:
            os.chmod(temporary, path.stat().st_mode)
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)
        return self.describe(name)
