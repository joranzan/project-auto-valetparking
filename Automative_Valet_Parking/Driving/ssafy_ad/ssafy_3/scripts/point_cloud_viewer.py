import numpy as np
import open3d as o3d
import time

if __name__ == "__main__":

    with open('./point_cloud2.txt', 'r') as f:
        pc = f.readlines()
    pc_list = [[float(y) for y in x.split("\n")[0].split(" ")] for x in pc[1:-8] if x != pc[-10]]
    
    xyz = np.array(pc_list)
    xyz
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    # print(pcd)
    # print(np.asarray(pcd.points))
    # o3d.visualization.draw_geometries([pcd])

    
    center_list = [[float(y) for y in x.split("\n")[0].split(" ")] for x in pc[-8:]]

    # pcd_center = o3d.geometry.PointCloud()
    # pcd_center.points = o3d.utility.Vector3dVector(center)

    # print(pcd_center)
    # print(np.asarray(pcd_center.points))
    # o3d.visualization.draw_geometries([pcd_center])

    vis = o3d.visualization.Visualizer()
    vis.create_window(height=480, width=640)

    vis.add_geometry(pcd)

    dt = 1
    previous_t = time.time()

    keep_running = True
    while keep_running:
        
        if time.time() - previous_t > dt:
            # Options (uncomment each to try them out):
            # 1) extend with ndarrays.
            center_list = [ [x[i]+dt if i == 2 else x[i] for i in range(len(x))] for x in center_list]
            center = np.array(center_list)
            pcd.points.extend(center)
            
            # 2) extend with Vector3dVector instances.
            # pcd.points.extend(
            #     o3d.utility.Vector3dVector(np.random.rand(n_new, 3)))
            
            # 3) other iterables, e.g
            # pcd.points.extend(np.random.rand(n_new, 3).tolist())
            
            vis.update_geometry(pcd)
            previous_t = time.time()

        keep_running = vis.poll_events()
        vis.update_renderer()

    vis.destroy_window()