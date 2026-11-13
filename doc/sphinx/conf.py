#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Shift documentation build configuration file

import os
import subprocess
import sys

import alabaster

sys.path.insert(0, os.path.abspath('.'))

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))

# -- General configuration ------------------------------------------------

extensions = [
    'sphinx.ext.mathjax',
    'sphinx.ext.todo',
    'sphinx.ext.extlinks',
    'breathe',
    'numfig',
    'sphinxcontrib.bibtex',
    'shift_parameters',
]

# Doxygen XML consumed by breathe. `meson compile -C build docs` sets
# SHIFT_DOXYGEN_XML to XML it has generated; otherwise the build runs doxygen
# itself (see setup below) into a directory next to the HTML output.
breathe_projects = {'shift': os.environ.get('SHIFT_DOXYGEN_XML', '')}
breathe_default_project = 'shift'
breathe_domain_by_extension = {'h': 'cpp', 'cc': 'cpp'}
breathe_default_members = ()

bibtex_bibfiles = ['../bibliography.bib']
bibtex_default_style = 'plain'

# links to files in the repository: :source:`examples/pendulum/pendulum.json`
github = 'https://github.com/shift-dynamics/shift'
github_branch = os.environ.get('SHIFT_DOC_BRANCH', 'master')
extlinks = {
    'source': (github + '/blob/' + github_branch + '/%s', '%s'),
    'tree': (github + '/tree/' + github_branch + '/%s', '%s'),
}

# the repository root, for literalinclude and shift-parameters
shift_root = ROOT

templates_path = ['_templates']
source_suffix = '.rst'
master_doc = 'index'

project = 'Shift'
copyright = '2015-2026, Kristopher Wehage'
version = '0.2'
release = '0.2.0'
today_fmt = '%B %d, %Y'

# software_design includes pages and figures that are not in the repository
exclude_patterns = ['_build', 'software_design',
                    'dynamics/handling_multi_dof_joints.rst']

pygments_style = 'trac'
highlight_language = 'javascript'  # model files are JSON with // comments

# -- Options for HTML output ----------------------------------------------

html_theme = 'alabaster'
html_theme_options = {
    'logo_name': True,
    'description': 'Shift: A Dynamics Engine and Visualization Tool for '
                   'Automation, Robotics and Vehicle Dynamics',
    'fixed_sidebar': True,
}
html_theme_path = [alabaster.get_path()]
html_short_title = 'Home'
html_static_path = ['_static']
html_css_files = ['custom.css']
html_last_updated_fmt = '%b %d, %Y'
html_sidebars = {
    '**': [
        'about.html',
        'navigation.html',
        'relations.html',
        'searchbox.html',
    ]
}
html_show_sphinx = False
htmlhelp_basename = 'shift-doc'

# -- Options for LaTeX output ---------------------------------------------

latex_elements = {
    'papersize': 'letterpaper',
    'pointsize': '10pt',
    'preamble': '\\usepackage{cancel}',
}
latex_documents = [
    ('index', 'shift.tex', 'Shift', 'Kristopher Wehage', 'manual'),
]
man_pages = [('index', 'shift', 'Shift Documentation', ['Kristopher Wehage'],
              1)]


def run_doxygen(app):
    """Generate the Doxygen XML for breathe unless it was given."""
    if app.config.breathe_projects.get('shift'):
        return
    out = os.path.join(os.path.dirname(app.outdir), 'doxygen')
    os.makedirs(out, exist_ok=True)
    env = dict(os.environ, SHIFT_ROOT=ROOT, SHIFT_DOXYGEN_OUT=out)
    # doxygen's warnings about the comments go to a log, not the console
    with open(os.path.join(out, 'doxygen.log'), 'w') as log:
        subprocess.run(['doxygen', os.path.join(HERE, 'Doxyfile')], cwd=HERE,
                       env=env, check=True, stdout=log, stderr=log)
    app.config.breathe_projects['shift'] = os.path.join(out, 'xml')


def setup(app):
    app.connect('builder-inited', run_doxygen)
