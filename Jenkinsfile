pipeline {
  agent any
  stages {
    stage('Build') {
      steps {
        echo 'Building..'
        sh 'touch build && rm -r build && mkdir -p build && cd build && cmake .. && make'
      }
    }
  }
}
