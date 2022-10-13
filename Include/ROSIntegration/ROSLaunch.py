import subprocess

class CROSLaunch():
    def __init__(self, astrPackage: str, astrLaunchFile: str):
        super(CROSLaunch, self).__init__()
        self._package = astrPackage
        self._launchFile = astrLaunchFile
        self._nav_proc: subprocess.Popen
        print("_package", "_launchFile")
    #'launch kobuki_nav kobuki_nav_launch.py'
    def launch(self):
        cmd = ['ros2', 'launch', self._package, self._launchFile]
        try:
            #self._nav_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL)
            self._nav_proc = subprocess.Popen(cmd)

        except :
            pass

    def terminate(self):
        self._nav_proc.terminate()

