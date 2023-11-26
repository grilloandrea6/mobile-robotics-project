from local_navigation import Local_Navigation



local_nav = Local_Navigation([(0,0)])
vl,vr = 50,50

for i in range(300):
      vl,vr = local_nav.obstacle_avoidance([1200, 1816, 0, 0, 0, 0, 0], [vl,vr])
      
      print(vl,vr)
    