pipeline {
    checkout([
         $class: 'GitSCM',
         branches: scm.branches,
         doGenerateSubmoduleConfigurations: scm.doGenerateSubmoduleConfigurations,
         extensions: scm.extensions,
         userRemoteConfigs: scm.userRemoteConfigs
    ])
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
