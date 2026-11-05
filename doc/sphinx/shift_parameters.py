"""A table of the JSON keys of a Shift object, read from its source.

Every Shift object declares its JSON keys in its DeclareParameters function:

    DeclareParameter("radii", &radii_, "radius of each pulley ...", true)

The directive

    .. shift-parameters:: shift/core/force_module/belt.cc
       :function: Belt::DeclareParameters
       :exclude: name, type

reads those calls from the file (relative to the repository root) and
renders a table of each key, whether it is required, and its description,
so that the tables stay in step with the code. Without :function:, every
call in the file is listed. Several files may be given, separated by blanks,
for an object whose base class declares keys too.
"""

import os
import re

from docutils import nodes
from docutils.parsers.rst import Directive, directives

_STRING = re.compile(r'"((?:[^"\\]|\\.)*)"')


def _call_end(text, start):
    """Index after the parenthesis that closes the call opened at start."""
    depth, i = 1, start
    while depth and i < len(text):
        c = text[i]
        if c == '"':
            i += 1
            while text[i] != '"':
                i += 2 if text[i] == '\\' else 1
        elif c == '/' and text[i + 1] == '/':
            i = text.index('\n', i)
        elif c == '(':
            depth += 1
        elif c == ')':
            depth -= 1
        i += 1
    return i


def _split_arguments(text):
    """Split a call's arguments at the commas outside strings and brackets."""
    parts, depth, i, begin = [], 0, 0, 0
    while i < len(text):
        c = text[i]
        if c == '"':
            i += 1
            while text[i] != '"':
                i += 2 if text[i] == '\\' else 1
        elif c in '([{':
            depth += 1
        elif c in ')]}':
            depth -= 1
        elif c == ',' and depth == 0:
            parts.append(text[begin:i])
            begin = i + 1
        i += 1
    parts.append(text[begin:])
    return parts


def _unescape(text):
    return text.replace('\\"', '"').replace('\\\\', '\\')


def _function_body(text, name):
    """The text of the body of the function whose definition names name."""
    for match in re.finditer(re.escape(name) + r'\s*\([^)]*\)\s*(?:const\s*)?\{',
                             text):
        start = match.end()
        depth, i = 1, start
        while depth and i < len(text):
            if text[i] == '{':
                depth += 1
            elif text[i] == '}':
                depth -= 1
            i += 1
        return text[start:i - 1]
    return None


def read_parameters(path, function=None):
    """[(key, required, description)] of the DeclareParameter calls."""
    with open(path, encoding='utf-8') as f:
        text = f.read()
    if function:
        body = _function_body(text, function)
        if body is None:
            raise ValueError('no function %s in %s' % (function, path))
        text = body
    parameters = []
    for match in re.finditer(r'\bDeclareParameter\s*\(', text):
        end = _call_end(text, match.end())
        arguments = _split_arguments(text[match.end():end - 1])
        if len(arguments) < 3:
            continue
        key = _STRING.findall(arguments[0])
        if not key:
            continue
        description = ''.join(_STRING.findall(','.join(arguments[2:-1]) if
                                              len(arguments) > 3 else
                                              arguments[2]))
        required = len(arguments) > 3 and arguments[-1].strip() == 'true'
        parameters.append((_unescape(key[0]), required,
                           _unescape(description)))
    return parameters


class ShiftParameters(Directive):
    required_arguments = 1
    final_argument_whitespace = True
    option_spec = {
        'function': directives.unchanged,
        'exclude': directives.unchanged,
    }

    def run(self):
        env = self.state.document.settings.env
        root = env.config.shift_root
        exclude = {k.strip() for k in
                   self.options.get('exclude', '').split(',') if k.strip()}
        rows = []
        for name in self.arguments[0].split():
            path = os.path.join(root, name)
            env.note_dependency(path)
            try:
                rows += read_parameters(path, self.options.get('function'))
            except (OSError, ValueError) as error:
                raise self.error(str(error))
        rows = [r for r in rows if r[0] not in exclude]

        table = nodes.table(classes=['shift-parameters'])
        group = nodes.tgroup(cols=3)
        table += group
        for width in (20, 10, 70):
            group += nodes.colspec(colwidth=width)
        head = nodes.thead()
        group += head
        head += self._row(['key', 'required', 'description'], header=True)
        body = nodes.tbody()
        group += body
        for key, required, description in rows:
            body += self._row([nodes.literal(text=key),
                               'yes' if required else '', description])
        return [table]

    @staticmethod
    def _row(cells, header=False):
        row = nodes.row()
        for cell in cells:
            entry = nodes.entry()
            paragraph = nodes.paragraph()
            if isinstance(cell, nodes.Node):
                paragraph += cell
            else:
                paragraph += nodes.Text(cell)
            entry += paragraph
            row += entry
        return row


def setup(app):
    app.add_config_value('shift_root', '', 'env')
    app.add_directive('shift-parameters', ShiftParameters)
    return {'parallel_read_safe': True, 'parallel_write_safe': True}
