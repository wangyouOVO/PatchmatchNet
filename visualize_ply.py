from argparse import ArgumentParser
import open3d as o3d
import numpy as np
import os

if __name__ == "__main__":
    showSfM = False
    fused = r"/home/wt/Projects/PatchmatchNet/output/scan/fused.ply"
    pcd = o3d.io.read_point_cloud(fused)
    if showSfM:
        camera = r"/home/wt/Projects/PatchmatchNet/output/hello_cameras.ply"
        camerapcd = o3d.io.read_point_cloud(camera)
        points = r"/home/wt/Projects/PatchmatchNet/output/hello_points.ply"
        pointspcd = o3d.io.read_point_cloud(points)
        pcd = camerapcd+pointspcd

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    ctr = vis.get_view_control()
    opt = vis.get_render_option()
    opt.point_size = 1.0
    opt.background_color = np.array([1,1, 1.0])
    vis.add_geometry(pcd)
   
    vis.run()
    vis.destroy_window()
    
