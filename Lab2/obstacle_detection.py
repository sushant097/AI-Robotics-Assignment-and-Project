#!/usr/bin/python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

#!/home/sushant/anaconda3/bin/python3

# pip install pyflann-py3
# sudo pip3 install pyflann-py3
from pyflann import *
import time


# from scipy.spatial.distance import cdist
from sklearn.neighbors import KDTree

# utility function to publish a point cloud ROS message
def publish_cloud(cloud, timestamp, colors=None):
    pcMsg.header.seq = count_msg
    pcMsg.header.stamp = timestamp
    if colors is None:
        pcMsg.fields = [
            PointField('x',0,PointField.FLOAT32,1),
            PointField('y',4,PointField.FLOAT32,1),
            PointField('z',8,PointField.FLOAT32,1),
        ]
        pcMsg.point_step = 12
    else:
        pcMsg.fields = [
            PointField('x',0,PointField.FLOAT32,1),
            PointField('y',4,PointField.FLOAT32,1),
            PointField('z',8,PointField.FLOAT32,1),
            PointField('rgb',12,PointField.INT32,1),
        ]
        pcMsg.point_step = 16
    pcMsg.width = len(cloud)
    pcMsg.row_step = pcMsg.width * pcMsg.point_step
    if colors is None:
        pcMsg.data = cloud.tobytes()
    else:
        color_int = colors[:,0] << 16 | colors[:,1] << 8 | colors[:,2]
        combined = np.rec.fromarrays([cloud[:,0], cloud[:,1], cloud[:,2], color_int], names='x,y,z,c')
        pcMsg.data = combined.tobytes()
    pubCloud.publish(pcMsg)

# utility function to publish bounding boxes as a ROS message
def publish_boxes(boxes, timestamp):
    boxMsg.header.stamp = timestamp
    boxMsg.points = []
    for box in boxes:
        p1 = Point(*box[0,:])
        p2 = Point(*box[1,:])
        p3 = Point(*box[2,:])
        p4 = Point(*box[3,:])
        p5 = Point(*box[4,:])
        p6 = Point(*box[5,:])
        p7 = Point(*box[6,:])
        p8 = Point(*box[7,:])
        boxMsg.points.extend([p1,p2,p1,p3,p2,p4,p3,p4,p1,p5,p2,p6,p3,p7,p4,p8,p5,p6,p5,p7,p6,p8,p7,p8])
    pubBoxes.publish(boxMsg)

# TODO: implementation function to filter out points with Z value below a specified threshold
def filter_ground(cloud, ground_level=0):
    mask = cloud[:,2] >= ground_level
    filtered_pts = cloud[mask]
    return filtered_pts
    
# TODO: implementation function to filter out points further than a specified distance
def filter_by_distance(cloud, distance=10):
    # 4 dimension in the bag files; ignore last dimension
    # Reference point (origin in this case) 
    ref = np.array([0, 0, 0, 0])
    # Calculate distance of each point from reference 
    dist = np.linalg.norm(cloud - ref, axis=1)
 

    # Create mask for points within distance threshold
    mask = dist <= distance

    # Filter points 
    filtered_pts = cloud[mask]
    return filtered_pts


# TODO: implementation function to perform Euclidean clustering at a specified threshold in meters
def euclidean_clustering(cloud, threshold=0.5):
    """
    Perform Euclidean clustering on 3D points within distance threshold
    """
    num_points = len(cloud)
    cluster_labels = np.zeros(num_points, dtype=int)
    cluster_id = 1  # Start with cluster ID 1

    # Use a KDTree for efficient neighbor search within the threshold
    kdtree = KDTree(cloud)
    for i in range(num_points):
        if cluster_labels[i] != 0:
            continue

        queue = [i]
        cluster_labels[i] = cluster_id

        while queue:
            point_idx = queue.pop(0)

            
            neighbors = kdtree.query_radius([cloud[point_idx]], r=threshold)[0]

            for neighbor_idx in neighbors:
                if cluster_labels[neighbor_idx] == 0:
                    queue.append(neighbor_idx)
                    cluster_labels[neighbor_idx] = cluster_id

        cluster_id += 1

    return cluster_labels


def euclidean_clustering_accelerated(cloud, threshold=0.5):
    """
    Perform Accelerated Euclidean clustering on 3D points within distance threshold using FLANN.
    NumPy arrays to store cluster labels and visited points, which allows for efficient element-wise operations. This replaces the explicit loop over neighbors.
    flann.nn_radius to find neighbors within the given threshold for each point. 
    update cluster labels and mark visited points in a vectorized manner.
    """
    num_points = len(cloud)
    cluster_labels = np.zeros(num_points, dtype=int)
    cluster_id = 1  # Start with cluster ID 1

    # Create a FLANN index for efficient neighbor search within the threshold
    flann = FLANN()
    flann.build_index(cloud, algorithm='kdtree')

    # Initialize an array to keep track of visited points
    visited = np.zeros(num_points, dtype=bool)

    for i in range(num_points):
        if cluster_labels[i] != 0:
            continue

        # Initialize a queue for BFS traversal
        queue = [i]
        cluster_labels[i] = cluster_id

        while queue:
            point_idx = queue.pop(0) # Convert the queue to a NumPy array
            neighbors = flann.nn_radius(cloud[point_idx], threshold)[0]

            # Filter neighbors that have not been visited
            unvisited_neighbors = neighbors[~visited[neighbors]]

            # Mark all unvisited neighbors with the same cluster ID
            cluster_labels[unvisited_neighbors] = cluster_id

            # Mark all visited points
            visited[unvisited_neighbors] = True

            # Add unvisited neighbors to the queue
            queue.extend(unvisited_neighbors)

        cluster_id += 1

    return cluster_labels



# TODO: (extra credit) implementation function to filter clusters by number of points
def filter_clusters(cluster_labels, min_cluster_size=100):
    """
    Filter out clusters that have fewer than a certain number of points.

    Parameters:
    cluster_labels (numpy.ndarray): Array containing cluster labels for each point.
    min_cluster_size (int): Minimum number of points a cluster should have to be considered valid.

    Returns:
    numpy.ndarray: Filtered cluster labels.
    """
    unique_clusters, cluster_counts = np.unique(cluster_labels, return_counts=True)

    # Iterate through unique clusters and filter out small clusters
    for cluster_id, count in zip(unique_clusters, cluster_counts):
        if cluster_id == 0:
            continue  # Skip unassigned points

        if count < min_cluster_size:
            # Set cluster label to 0 for small clusters
            cluster_labels[cluster_labels == cluster_id] = 0

    return cluster_labels


# TODO: implementation function to compute bounding boxes from cluster labels
def get_bounding_boxes(cloud, cluster_labels):
    bounding_boxes = []
    unique_clusters = np.unique(cluster_labels) 
    
    for cluster_id in unique_clusters:
        if cluster_id ==0: # 0 is not the label
            continue
        cluster_points = cloud[cluster_labels == cluster_id]
        min_coords = np.min(cluster_points, axis=0)
        max_coords = np.max(cluster_points, axis=0)
        
        corners = np.array([
            [min_coords[0], min_coords[1], min_coords[2]],
            [max_coords[0], min_coords[1], min_coords[2]],
            [min_coords[0], max_coords[1], min_coords[2]],
            [max_coords[0], max_coords[1], min_coords[2]],
            [min_coords[0], min_coords[1], max_coords[2]],
            [max_coords[0], min_coords[1], max_coords[2]],
            [min_coords[0], max_coords[1], max_coords[2]],
            [max_coords[0], max_coords[1], max_coords[2]]
        ])
        
        bounding_boxes.append(corners)
        
    
    return bounding_boxes

# callback function to subscribe to the ROS point cloud message input
def point_cloud_callback(msg):
    global count_msg
    start_time = time.time()
    input_cloud = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)

    # Filter out ground points
    filtered_cloud = filter_ground(input_cloud, -1.2)

    # Filter out points that are out of range
    filtered_cloud = filter_by_distance(filtered_cloud, 10)

    # measure time taken
    start_time = time.time()
    # Determine cluster labels
    cluster_labels = euclidean_clustering(filtered_cloud[:, :3])
    # running optimize euclidean clustering 
    # cluster_labels = euclidean_clustering_accelerated(filtered_cloud[:, :3])
    end_time = time.time()
    print(f"Time taken to run accelerated euclidean_clustering: {end_time - start_time} s")


    # Filter out clusters that are too small
    cluster_labels = filter_clusters(cluster_labels, 100)

    # Set a unique color for each cluster
    colors = np.zeros((len(filtered_cloud), 3), dtype=np.int32)
    for i in range(cluster_labels.min(), cluster_labels.max()+1):
        cluster_mask = cluster_labels == i
        colors[cluster_mask] = np.random.randint(0,255,3)

    # Compute a bounding box for each cluster
    boxes = get_bounding_boxes(filtered_cloud, cluster_labels)
    publish_boxes(boxes, msg.header.stamp)

    publish_cloud(filtered_cloud[:, :3], msg.header.stamp, colors)
    count_msg += 1
    end_time = time.time()
    print("Message %d: Processed %d points %d clusters in %.3fs" % (count_msg, len(input_cloud), len(set(cluster_labels) - set([0])), end_time - start_time))

rospy.init_node('obstacle_detection', anonymous=True)
count_msg = 0
rospy.Subscriber("/kitti/velo/pointcloud", PointCloud2, point_cloud_callback)

# initialize the output ROS messages
pcMsg = PointCloud2()
pcMsg.header.frame_id = 'velo_link'
pcMsg.height = 1
pcMsg.is_bigendian = False
pcMsg.is_dense = True
pubCloud = rospy.Publisher('output_cloud', PointCloud2, queue_size=1)

boxMsg = Marker()
boxMsg.header.frame_id = "velo_link";
boxMsg.type = Marker.LINE_LIST;
boxMsg.lifetime = rospy.Duration();
boxMsg.color.a = 1.0;
boxMsg.action = Marker.ADD;
boxMsg.scale.x = 0.05;
boxMsg.pose.orientation.w = 1.0;
boxMsg.id = 0;
boxMsg.color.r = 1.0;
boxMsg.color.g = 1.0;
boxMsg.color.b = 0;
pubBoxes = rospy.Publisher('boxes', Marker, queue_size=1)

rospy.spin()
