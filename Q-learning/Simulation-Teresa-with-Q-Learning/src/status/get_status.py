#from __future__ import print_function
#import roslibpy

f= open("guru99.txt","w+")

for i in range(10):
     f.write("This is line %d\r\n" % (i+1))

f.close() 

#client = roslibpy.Ros(host='localhost', port=9090)
#client.run()



#listener = roslibpy.Topic(client, '/bebop2/states/common/CommonState/BatteryStateChanged', 'bebop_msgs/CommonCommonStateBatteryStateChanged')
#listener = roslibpy.Topic(client, '/bebop2/odom', 'nav_msgs/Odometry')
#listener.subscribe(lambda data: print(data['data']))

#try:
#     while True:
#         pass
# except KeyboardInterrupt:
#     client.terminate()
