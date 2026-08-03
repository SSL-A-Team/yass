import sysconfig
from setuptools import setup, Extension
from Cython.Build import cythonize

extra_compile_args = sysconfig.get_config_var('CFLAGS').split()
extra_link_args = sysconfig.get_config_var('LDFLAGS').split()
extra_link_args.append(f"-lpython{sysconfig.get_config_var('LDVERSION')}")

ext_modules = [
    Extension(
        name="libexample_python_plugin",
        sources=["plugin.pyx"],
        extra_compile_args=extra_compile_args,
        extra_link_args=extra_link_args,
    )
]

setup(
    name="libexample_python_plugin",
    ext_modules=cythonize(
        ext_modules,
        compiler_directives={'language_level': '3'}
    )
)
