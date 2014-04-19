import os
import subprocess
import sys
import shutil

import sphinx

import setuptools
import distutils
import numpy.distutils

try:
    from paver.tasks import VERSION as _PVER
    if not _PVER >= '1.0':
        raise RuntimeError("paver version >= 1.0 required (was %s)" % _PVER)
except ImportError, e:
    raise RuntimeError("paver version >= 1.0 required")

import paver
import paver.doctools
import paver.path
from paver.easy import options, Bunch, task, needs, dry, sh, call_task
from paver.setuputils import setup

import common

PDF_DESTDIR = paver.path.path('docs') / 'pdf'
HTML_DESTDIR = paver.path.path('docs') / 'html'

setup(name=common.DISTNAME,
        namespace_packages=['scikits'],
        packages=setuptools.find_packages(),
        install_requires=common.INSTALL_REQUIRE,
        version=common.VERSION,
        include_package_data=True)

options(sphinx=Bunch(builddir="build", sourcedir="src"),
        virtualenv=Bunch(script_name="install/bootstrap.py"))

def macosx_version():
    st = subprocess.Popen(["sw_vers"], stdout=subprocess.PIPE)
    out = st.stdout.readlines()
    import re
    ver = re.compile("ProductVersion:\s+([0-9]+)\.([0-9]+)\.([0-9]+)")
    for i in out:
        m = ver.match(i)
        if m:
            return m.groups()

def mpkg_name():
    maj, min = macosx_version()[:2]
    pyver = ".".join([str(i) for i in sys.version_info[:2]])
    return "scikits.samplerate-%s-py%s-macosx%s.%s.mpkg" % (common.build_fverstring(),
                            pyver, maj, min)

VPYEXEC = "install/bin/python"

@task
def bootstrap():
    """create virtualenv in ./install"""
    install = paver.path.path('install')
    if not install.exists():
        install.mkdir()
    call_task('paver.virtual.bootstrap')
    sh('cd install; %s bootstrap.py' % sys.executable)

@task
@needs('bootstrap')
def test_install():
    """Install the package into the venv."""
    sh('%s setup.py install' % VPYEXEC)

@task
def clean():
    """Remove build, dist, egg-info garbage."""
    d = ['build', 'dist', 'scikits.samplerate.egg-info', HTML_DESTDIR,
            PDF_DESTDIR]
    for i in d:
        paver.path.path(i).rmtree()

    (paver.path.path('docs') / options.sphinx.builddir).rmtree()

@task
def clean_bootstrap():
    paver.path.path('install').rmtree()

@task
@needs("setuptools.bdist_mpkg", "doc")
def dmg():
    pyver = ".".join([str(i) for i in sys.version_info[:2]])
    builddir = paver.path.path("build") / "dmg"
    builddir.rmtree()
    builddir.mkdir()

    # Copy mpkg into image source
    mpkg_n = mpkg_name()
    mpkg = paver.path.path("dist") / mpkg_n
    mpkg.copytree(builddir / mpkg_n)
    tmpkg = builddir / mpkg_n
    tmpkg.rename(builddir / ("samplerate-%s-py%s.mpkg" % (common.build_fverstring(), pyver)))

    # Copy docs into image source
    doc_root = paver.path.path(builddir) / "docs"
    html_docs = paver.path.path("docs") / "html"
    pdf_docs = paver.path.path("docs") / "pdf" / "samplerate.pdf"
    html_docs.copytree(doc_root / "html")
    pdf_docs.copy(doc_root / "samplerate.pdf")

    # Build the dmg
    image_name = "samplerate-%s.dmg" % common.build_fverstring()
    image = paver.path.path(image_name)
    image.remove()
    cmd = ["hdiutil", "create", image_name, "-srcdir", str(builddir)]
    sh(" ".join(cmd))
#options.setup.package_data =
#    setuputils.find_package_data("scikits/samplerate",
#                                 package="scikits/samplerate",
#                                 only_in_packages=False)

if paver.doctools.has_sphinx:
    def _latex_paths():
        """look up the options that determine where all of the files are."""
        opts = options
        docroot = paver.path.path(opts.get('docroot', 'docs'))
        if not docroot.exists():
            raise BuildFailure("Sphinx documentation root (%s) does not exist."
                    % docroot)
        builddir = docroot / opts.get("builddir", ".build")
        builddir.mkdir()
        srcdir = docroot / opts.get("sourcedir", "")
        if not srcdir.exists():
            raise BuildFailure("Sphinx source file dir (%s) does not exist"
                    % srcdir)
        latexdir = builddir / "latex"
        latexdir.mkdir()
        return Bunch(locals())

    @task
    @needs('build_version_files')
    def latex():
        """Build samplerate's documentation and install it into
        scikits/samplerate/docs"""
        paths = _latex_paths()
        sphinxopts = ['', '-b', 'latex', paths.srcdir, paths.latexdir]
        dry("sphinx-build %s" % (" ".join(sphinxopts),), sphinx.main, sphinxopts)

    @task
    @needs('latex')
    def pdf():
        paths = _latex_paths()
        def build_latex():
            subprocess.call(["make", "all-pdf"], cwd=paths.latexdir)
        dry("Build pdf doc", build_latex)
        PDF_DESTDIR.rmtree()
        PDF_DESTDIR.makedirs()
        pdf = paths.latexdir / "samplerate.pdf"
        pdf.copy(PDF_DESTDIR)

    @task
    @needs('build_version_files', 'paver.doctools.html')
    def html():
        """Build samplerate documentation and install it into docs"""
        builtdocs = paver.path.path("docs") / options.sphinx.builddir / "html"
        HTML_DESTDIR.rmtree()
        builtdocs.copytree(HTML_DESTDIR)

    @task
    @needs(['html', 'pdf'])
    def doc():
        pass

    @task
    def build_version_files(options):
        from common import write_version
        write_version(os.path.join("scikits", "samplerate", "version.py"))
        if os.path.exists(os.path.join("docs", "src")):
            write_version(os.path.join("docs", "src", "samplerate_version.py"))
    @task
    @needs('html', 'pdf', 'setuptools.command.sdist')
    def sdist(options):
        """Build tarball."""
        pass
