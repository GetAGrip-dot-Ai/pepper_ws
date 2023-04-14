import sys
import roslaunch

# print(sys.argv)

arg_list = sys.argv
print("arg_list: ", arg_list)

# node = roslaunch.core.Node(package, executable, poi=arg_list)


package = 'pepper_ws'
executable = 'MarkerPublisher.py'

node = roslaunch.core.Node(package, executable)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(node)
print (process.is_alive())
process.stop()