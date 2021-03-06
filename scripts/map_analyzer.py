#!/usr/bin/python

from PIL import Image
import numpy as np
import rospy
from project_utils import INDEX_FOR_X, INDEX_FOR_Y, pixel2pose, FREE, OCCUPIED, save_data, get_point
from nav_msgs.msg import OccupancyGrid
from gvgexplore.msg import Coverage
from std_msgs.msg import String


class MapAnalyzer:
    def __init__(self):
        rospy.init_node('map_analyzer', anonymous=True)
        self.robot_count = rospy.get_param("~robot_count")
        self.scale = rospy.get_param("~map_scale")
        self.map_file_name = rospy.get_param("~map_file")
        self.run = rospy.get_param("~run")
        self.debug_mode = rospy.get_param("~debug_mode")
        self.termination_metric = rospy.get_param("~termination_metric")
        self.environment = rospy.get_param("~environment")
        self.free_pixel_ratio = self.read_raw_image()
        self.all_coverage_data = []
        self.coverage_pub = rospy.Publisher("/coverage", Coverage, queue_size=10)
        rospy.Subscriber('/shutdown', String, self.shutdown_callback)
        rospy.on_shutdown(self.save_all_data)
        self.robot_maps = {}
        for i in range(self.robot_count):
            exec("def a_{0}(self, data): self.robot_maps[{0}] =data".format(i))
            exec("setattr(MapAnalyzer, 'callback_map{0}', a_{0})".format(i))
            exec("rospy.Subscriber('/robot_{0}/map', OccupancyGrid, self.callback_map{0}, "
                 "queue_size = 1)".format(i))

    def spin(self):
        r = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            self.publish_coverage()
            r.sleep()

    def publish_coverage(self):
        all_explored_points = {}
        allpoints = {}
        common_points = []
        for rid in range(self.robot_count):
            if rid in self.robot_maps:
                map = self.robot_maps[rid]  # rospy.wait_for_message('/robot_{}/map'.format(rid), OccupancyGrid)
                rid_coverage, rid_unknown = self.get_map_description(map)
                cov_set = set(list(rid_coverage.keys()))
                common_points.append(cov_set)
                all_explored_points.update(rid_coverage)
                allpoints.update(rid_coverage)
                allpoints.update(rid_unknown)
        if allpoints:
            N = float(len(allpoints))
            common_area = set.intersection(*common_points)
            covered_area = len(all_explored_points)
            cov_ratio = covered_area / N
            common_coverage = float(len(common_area)) / N
            cov_msg = Coverage()
            cov_msg.header.stamp = rospy.Time.now()
            cov_msg.coverage = cov_ratio
            cov_msg.expected_coverage = self.free_pixel_ratio
            cov_msg.common_coverage = common_coverage
            self.coverage_pub.publish(cov_msg)
            self.all_coverage_data.append(
                {'time': rospy.Time.now().to_sec(), 'explored_ratio': cov_ratio, 'common_coverage': common_coverage,
                 'expected_coverage': self.free_pixel_ratio})

    def get_map_description(self, occ_grid):
        resolution = occ_grid.info.resolution
        origin_pos = occ_grid.info.origin.position
        origin_x = origin_pos.x
        origin_y = origin_pos.y
        height = occ_grid.info.height
        width = occ_grid.info.width
        grid_values = np.array(occ_grid.data).reshape((height, width)).astype(np.float32)
        num_rows = grid_values.shape[0]
        num_cols = grid_values.shape[1]
        covered_area = {}
        unknown = {}
        for row in range(num_rows):
            for col in range(num_cols):
                index = [0] * 2
                index[INDEX_FOR_Y] = num_rows - row - 1
                index[INDEX_FOR_X] = col
                index = tuple(index)
                pose = pixel2pose(index, origin_x, origin_y, resolution)
                pose = get_point(pose)
                p = grid_values[num_rows - row - 1, col]
                if p == FREE:
                    covered_area[pose] = None
                else:
                    unknown[pose] = None
        return covered_area, unknown

    def read_raw_image(self):
        im = Image.open(self.map_file_name, 'r')
        basewidth = im.size[0] // self.scale
        wpercent = (basewidth / float(im.size[0]))
        hsize = int((float(im.size[1]) * float(wpercent)))
        im = im.resize((basewidth, hsize), Image.ANTIALIAS)
        pixelMap = im.load()
        img = Image.new(im.mode, im.size)
        free_point_count = 0
        allpixels = 0
        for i in range(img.size[0]):
            for j in range(img.size[1]):
                pixel = pixelMap[i, j][0]
                allpixels += 1
                if pixel >= 199:
                    free_point_count += 1
        ratio = free_point_count / float(allpixels)
        return ratio

    def shutdown_callback(self, msg):
        rospy.signal_shutdown('MapAnalyzer: Shutdown command received!')

    def save_all_data(self):
        save_data(self.all_coverage_data,
                  'gvgexplore/coverage_{}_{}_{}_{}.pickle'.format(self.environment, self.robot_count, self.run,
                                                           self.termination_metric))


if __name__ == '__main__':
    analyzer = MapAnalyzer()
    analyzer.spin()
