import os
import subprocess
import sys
import tarfile
from pathlib import Path
from setuptools import Extension, setup, find_packages 
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):
    """A custom extension for CMake-based projects."""
    def __init__(self, name: str, sourcedir: str = "", **kwargs):
        super().__init__(name, sources=[], **kwargs)
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    """Custom build_ext command to run CMake."""
    def build_extension(self, ext: CMakeExtension):
        import numpy as np 
        import sysconfig

        """ The destination directory for the compiled extension.
                self.get_ext_fullpath(ext.name) will return something like:
                    build/lib.linux-x86_64-3.9/pywrapped_svo/_core.so
                .parent.resolve() gives us the final package directory:
                    build/lib.linux-x86_64-3.9/pywrapped_svo/ 
        """
        extdir = Path(self.get_ext_fullpath(ext.name)).parent.resolve()
        python_include_dir = sysconfig.get_path("include")
        numpy_include_dir = np.get_include()

        # Allow user to override build type with an environment variable
        build_type = os.environ.get("CMAKE_BUILD_TYPE", "Release")  # or Debug if needed
        
        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}{os.sep}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            f"-DCMAKE_BUILD_TYPE={build_type}",
            f"-DCMAKE_CXX_FLAGS=-I{numpy_include_dir} -I{python_include_dir}", 
            # --- Debug Use Only ---
            # f"-DCMAKE_CXX_FLAGS={os.environ.get('CMAKE_CXX_FLAGS', '')} -fsanitize=address -g -I{numpy_include_dir}",   
            # f"-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address",
            # f"-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=address",
            # ------
            f"-Wno-dev -Wno-deprecated-declarations -Wno-maybe-uninitialized" # Suppress CMake developer warnings
        ]

        # Allow user to pass extra CMake args
        if "CMAKE_ARGS" in os.environ:
            cmake_args += os.environ["CMAKE_ARGS"].split()
            
        build_temp = Path(self.build_temp)
        build_temp.mkdir(parents=True, exist_ok=True)

        print(f"Configuring CMake project with: {' '.join(cmake_args)}")
        subprocess.check_call(["cmake", str(ext.sourcedir)] + cmake_args, cwd=build_temp)

        # Allow user to override parallel jobs 
        build_jobs = os.environ.get("CMAKE_BUILD_PARALLEL_LEVEL")
        if not build_jobs:
            build_jobs = str(os.cpu_count() or 8) # Default to number of CPUs
        
        print(f"Building project with {build_jobs} parallel jobs")
        subprocess.check_call(
            ["cmake", "--build", ".", "--parallel", build_jobs],
            cwd=build_temp
        )

setup(
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    ext_modules=[CMakeExtension("svo_cpp._core", sourcedir=".")], 
    cmdclass={"build_ext": CMakeBuild},
    package_data={
        'svo_cpp': ['*.so', '*.pyd', '*.dylib'], 
    },
    include_package_data=True,
    zip_safe=False,
)