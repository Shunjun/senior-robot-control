pipeline {
    agent any
    options { disableConcurrentBuilds(); timestamps() }
    triggers { githubPush() }
    parameters {
        string(name: 'ENV_CREDENTIAL_ID', defaultValue: 'senior-control-env', description: 'Jenkins Secret file credential ID')
        string(name: 'DEPLOY_ROOT', defaultValue: '/opt/senior-robot-control', description: 'Persistent absolute deployment directory on this node')
    }
    stages {
        stage('Build web') {
            steps {
                dir('web') {
                    sh 'npm ci --include=dev'
                    sh 'npm run typecheck'
                    sh 'VITE_DEMO_DATA=0 npm run build'
                }
            }
        }
        stage('Deploy on this node') {
            steps {
                withCredentials([file(credentialsId: params.ENV_CREDENTIAL_ID, variable: 'ENV_FILE')]) {
                    withEnv(["DEPLOY_ROOT=${params.DEPLOY_ROOT}", "RELEASE_ID=${env.GIT_COMMIT}-${env.BUILD_NUMBER}"]) {
                        sh 'set +x; bash deploy/jenkins-deploy.sh'
                    }
                }
            }
        }
    }
}
