// ManipulatorCore CI (declarative pipeline, Linux agent with GCC/Clang, CMake >= 3.21, Ninja,
// clang-format/clang-tidy and Python 3.11+). Expensive steps are opt-in through parameters.

pipeline {
  agent any

  options {
    disableConcurrentBuilds()
    timeout(time: 120, unit: 'MINUTES')
    buildDiscarder(logRotator(numToKeepStr: '20'))
    timestamps()
  }

  parameters {
    booleanParam(name: 'RENDER_DOCS', defaultValue: false,
                 description: 'Regenerate the animations and figures in docs/assets (slow, CPU only)')
    booleanParam(name: 'RUN_BENCHMARKS', defaultValue: false,
                 description: 'Run the kernel and metric benchmarks and archive docs/benchmarks/README.md')
    booleanParam(name: 'BUILD_ROS2', defaultValue: false,
                 description: 'Build and test the optional ROS 2 package in a ros:jazzy container (needs Docker)')
  }

  environment {
    // A 16 GB machine: keep compile and test parallelism moderate.
    CMAKE_BUILD_PARALLEL_LEVEL = '4'
    CTEST_PARALLEL_LEVEL = '4'
    CMAKE_GENERATOR = 'Ninja'
    PYTHONDONTWRITEBYTECODE = '1'
  }

  stages {
    stage('Checkout') {
      steps {
        checkout scm
      }
    }

    stage('Environment') {
      steps {
        sh '''
          set -eu
          cmake --version
          ninja --version
          ${CXX:-c++} --version
          python3 --version
          clang-format --version || true
          clang-tidy --version || true
        '''
      }
    }

    stage('CMake Configure') {
      steps {
        sh 'cmake --preset debug'
      }
    }

    stage('Debug Build') {
      steps {
        sh 'cmake --build --preset debug'
      }
    }

    stage('Unit Tests') {
      steps {
        sh 'ctest --test-dir build/debug -L unit --output-on-failure --output-junit unit-tests.xml'
      }
    }

    stage('Numerical Tests') {
      steps {
        sh 'ctest --test-dir build/debug -L numerical --output-on-failure --output-junit numerical-tests.xml'
      }
    }

    stage('Integration Tests') {
      steps {
        sh 'ctest --test-dir build/debug -L integration --output-on-failure --output-junit integration-tests.xml'
      }
    }

    stage('Release Build') {
      steps {
        sh '''
          set -eu
          cmake --preset release
          cmake --build --preset release
          ctest --test-dir build/release --output-on-failure --output-junit release-tests.xml
        '''
      }
    }

    stage('clang-format') {
      steps {
        sh "git ls-files '*.hpp' '*.cpp' | xargs clang-format --dry-run --Werror"
      }
    }

    stage('clang-tidy') {
      steps {
        sh '''
          set -eu
          git ls-files 'src/*.cpp' | xargs -P 4 -n 4 clang-tidy -p build/debug --quiet --warnings-as-errors='*'
        '''
      }
    }

    stage('Python Ruff') {
      steps {
        sh '''
          set -eu
          python3 -m venv .ci-venv
          . .ci-venv/bin/activate
          pip install --quiet --upgrade pip
          pip install --quiet -e ".[dev]"
          ruff check python scripts
          ruff format --check python scripts
        '''
      }
    }

    stage('Python Tests') {
      steps {
        sh '''
          set -eu
          . .ci-venv/bin/activate
          pytest --junitxml=python-tests.xml
          python scripts/docs/generate_status_matrix.py --check
        '''
      }
    }

    stage('Sanitizer Build') {
      steps {
        sh '''
          set -eu
          cmake --preset asan-ubsan
          cmake --build --preset asan-ubsan
          ASAN_OPTIONS=detect_leaks=1:strict_string_checks=1 UBSAN_OPTIONS=print_stacktrace=1 \
            ctest --test-dir build/asan-ubsan --output-on-failure --output-junit sanitizer-tests.xml
        '''
      }
    }

    stage('Demo Smoke Tests') {
      steps {
        sh '''
          set -eu
          ctest --test-dir build/release -L smoke --output-on-failure
          build/release/bin/mc_demos build/demo_data
        '''
      }
    }

    stage('Install/Package Test') {
      steps {
        sh '''
          set -eu
          rm -rf "$WORKSPACE/_install" build/consumer
          cmake --install build/release --prefix "$WORKSPACE/_install"
          cmake -S examples/external_project -B build/consumer -DCMAKE_PREFIX_PATH="$WORKSPACE/_install"
          cmake --build build/consumer
          build/consumer/consumer
        '''
      }
    }

    stage('Benchmarks') {
      when { expression { params.RUN_BENCHMARKS } }
      steps {
        sh '''
          set -eu
          . .ci-venv/bin/activate
          python scripts/benchmark/run_benchmarks.py --build build/release
        '''
        archiveArtifacts artifacts: 'docs/benchmarks/README.md', fingerprint: true
      }
    }

    stage('Render Docs') {
      when { expression { params.RENDER_DOCS } }
      steps {
        sh '''
          set -eu
          . .ci-venv/bin/activate
          python scripts/render/render_all.py --demos build/release/bin/mc_demos --skip-demos --data build/demo_data
        '''
        archiveArtifacts artifacts: 'docs/assets/*', fingerprint: true
      }
    }

    stage('ROS 2 Package') {
      when { expression { params.BUILD_ROS2 } }
      steps {
        sh 'scripts/ci/ros2_docker_build.sh'
      }
    }
  }

  post {
    always {
      junit allowEmptyResults: true, testResults: 'build/**/*-tests.xml, python-tests.xml'
    }
    cleanup {
      sh 'rm -rf .ci-venv _install'
    }
  }
}
