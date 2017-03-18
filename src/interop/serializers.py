# -*- coding: utf-8 -*-

"""Interoperability API message serializer."""

import cv2
import utm
import rospy
import numpy as np
import dateutil.parser
from dateutil.tz import tzutc
from datetime import datetime
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped, PolygonStamped, Polygon
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Float64, Header, String, Time, Int16
from interop.msg import (Color, FlyZone, FlyZoneArray, 
                         Orientation, Shape, Target, TargetType)


def meters_to_feet(m):
    """Converts a distance from meters to feet.

    Args:
        m: Distance in meters.

    Returns:
        Distance in feet.
    """
    return float(m) / 0.3048


def feet_to_meters(ft):
    """Converts a distance from feet to meters.

    Args:
        ft: Distance in feet.

    Returns:
        Distance in meters.
    """
    return float(ft) * 0.3048


def iso8601_to_rostime(iso):
    """Converts ISO 8601 time to ROS Time.

    Args:
        iso: ISO 8601 encoded string.

    Returns:
        std_msgs/Time.
    """
    # Convert to datetime in UTC.
    t = dateutil.parser.parse(iso)
    if not t.utcoffset():
        t = t.replace(tzinfo=tzutc())

    # Convert to time from epoch in UTC.
    epoch = datetime.utcfromtimestamp(0)
    epoch = epoch.replace(tzinfo=tzutc())
    dt = t - epoch

    # Create ROS message.
    time = Time()
    time.data.secs = int(dt.total_seconds())
    time.data.nsecs = dt.microseconds * 1000

    return time


class MissionDeserializer(object):

    """Mission information deserializer."""

    @classmethod
    def __get_flyzone(cls, json, frame):
        """
        Deserializes flight boundary data into a FlyZoneArray message.
            
        Args:
            json: List of JSON dicts.
            frame: Frame id for the boundaries.

        Returns:
            A FlyzoneArray message type which contains an array of FlyZone
            messages, which contains a polygon for the boundary, a max 
            altitude and a min altitude.
        """
        # Generate header for all zones.
        header = Header()
        header.stamp = rospy.get_rostime()
        header.frame_id = frame

        flyzones = FlyZoneArray()
        for zone in json:
            flyzone = FlyZone()
            flyzone.zone.header = header

            flyzone.max_alt = feet_to_meters(zone["altitude_msl_max"])
            flyzone.min_alt = feet_to_meters(zone["altitude_msl_min"])

            # Change boundary points to ros message of type polygon.
            for waypoint in zone["boundary_pts"]:
                point = Point()
                easting, northing, _, _ = utm.from_latlon(
                                                      waypoint["latitude"],
                                                      waypoint["longitude"])
                point.x = easting
                point.y = northing
                flyzone.zone.polygon.points.append(point)

            flyzones.flyzones.append(flyzone)

        return flyzones

    @classmethod
    def __get_waypoints(cls, json, frame):
        """
        Deserializes a list of waypoints into a marker message.

        Args:
            json: List of JSON Dicts corresponding to waypoints.
            frame: Frame of the markers.

        Returns:
            A marker message of type Points, with a list of points in order
            corresponding to each waypoint.
        """
        header = Header()
        header.stamp = rospy.get_rostime()
        header.frame_id = frame

        waypoints = Marker()
        waypoints.header = header
        waypoints.ns = "waypoints"
        waypoints.type = Marker.POINTS

        # Set lifetime to 0 because waypoints do not change.
        waypoints.lifetime = rospy.Duration()

        # Ensure there is no rotation by setting w to 1.
        waypoints.pose.orientation.w = 1.0
        waypoints.scale.x = waypoints.scale.y = waypoints.scale.z = 0.1
        for point in json:
            waypoint = Point()
            easting, northing, _, _ = utm.from_latlon(point["latitude"],
                                                      point["longitude"])
            altitude = feet_to_meters(point["altitude_msl"])
            
            waypoint.x = easting
            waypoint.y = northing
            waypoint.z = altitude

            waypoints.points.append(waypoint)

        return waypoints

    @classmethod
    def __get_search_grid(cls, json, frame):
        """
        Deserializes a the search grid into a polygon message.

        Args:
            json: List of JSON dicts corresponding to the search grid points.
            frame: Frame for the polygon.

        Returns:
            Message of type PolygonStamped with the bounds of the search grid.
        """
        header = Header()
        header.stamp = rospy.get_rostime()
        header.frame_id = frame

        search_grid = PolygonStamped()
        search_grid.header = header

        for point in json:
            boundary_pnt = Point()

            easting, northing, _, _ = utm.from_latlon(point["latitude"],
                                                      point["longitude"])
            altitude = feet_to_meters(point["altitude_msl"])
            
            boundary_pnt.x = easting
            boundary_pnt.y = northing
            boundary_pnt.z = altitude

            search_grid.polygon.points.append(boundary_pnt)

        return search_grid

    @classmethod
    def __get_airdrop_loc(cls, json, frame):
        """
        Deserializes the airdrop location to a ros message of type 
        PointStamped.

        Args:
            json: A JSON dict with the air drop location.
            frame: Frame for the point.

        Returns:
            A PointStamped message with x and y corresponding to the airdrop
            location.
        """
        header = Header()
        header.stamp = rospy.get_rostime()
        header.frame_id = frame

        air_drop = PointStamped()
        air_drop.header = header

        easting, northing, _, _ = utm.from_latlon(json["latitude"],
                                                  json["longitude"])
        air_drop.point.x = easting
        air_drop.point.y = northing

        return air_drop

    @classmethod
    def __get_offaxis_targ(cls, json, frame):
        """
        Deserializes off axis target location to a message of type PointStamped.

        Args:
            json: JSON dict containing the off axis target location.
            frame: Frame for the point.

        Returns:
            A message of type PointStamped with the location of the off axis
            target.
        """
        header = Header()
        header.stamp = rospy.get_rostime()
        header.frame_id = frame
        
        off_axis_targ = PointStamped()
        off_axis_targ.header = header

        easting, northing, _, _ = utm.from_latlon(json["latitude"],
                                                  json["longitude"])

        off_axis_targ.point.x = easting
        off_axis_targ.point.y = northing

        return off_axis_targ

    @classmethod
    def __emergent_object(cls, json, frame):
        """
        Deserializes the last known location of the emergent target to a 
        ros message of type PointStamped.

        Args:
            json: JSON dict with the corresponding location.
            frame: Frame for the point.

        Returns:
            A PointStamped message with the information about the last known
            location of the emergent target.
        """
        header = Header()
        header.stamp = rospy.get_rostime()
        header.frame_id = frame

        emergent_obj = PointStamped()
        emergent_obj.header = header

        easting, northing, _, _ = utm.from_latlon(json["latitude"],
                                                  json["longitude"])

        emergent_obj.point.x = easting
        emergent_obj.point.y = northing

        return emergent_obj
    
    @classmethod
    def __utm_zone(cls, json):
        ref_zone = String()
        _, _, letter, num = utm.from_latlon(json["latitude"], 
                                            json["longitude"])
        data = "{0}{1}".format(letter, num)
        ref_zone.data = data

        return ref_zone

    @classmethod
    def from_json(cls, json, frame):
        """
        Deserializes the mission object from json to several ros messages.

        Args:
            json: JSON Dict.
            frame: frame id for the messages.

        Returns:
            A tuple of (FlyZoneArray, PolygonStamped, Marker, PointStamped,
            PointStamped, PointStamped) corresponding to the flyzones, search
            grid, waypoints, air drop position, off axis target location, and
            the emergent object location.
        """
        flyzones = cls.__get_flyzone(json["fly_zones"], frame)
        search_grid = cls.__get_search_grid(json["search_grid_points"], frame)
        waypoints = cls.__get_waypoints(json["mission_waypoints"], frame)
        air_drop_pos = cls.__get_airdrop_loc(json["air_drop_pos"], frame)
        off_axis_targ = cls.__get_offaxis_targ(json["off_axis_target_pos"], 
                                               frame)
        emergent_obj = cls.__emergent_object(json["emergent_last_known_pos"],
                                             frame)
        utm_zone = cls.__utm_zone(json["home_pos"])
        return (flyzones, search_grid, waypoints, air_drop_pos,
                off_axis_targ, emergent_obj, utm_zone)


class ObstaclesDeserializer(object):

    """Obstacles message deserializer."""

    # McGill Robotics red, because we have big egos.
    OBSTACLE_COLOR = ColorRGBA(
        r=218 / 255.0,
        g=41 / 255.0,
        b=28 / 255.0,
        a=1.0)

    @classmethod
    def from_json(cls, json, frame, lifetime):
        """Deserializes obstacle data into two MarkerArrays.

        Args:
            json: JSON dictionary.
            frame: Frame ID of every Marker.
            lifetime: Lifetime of every Marker in seconds.

        Returns:
            Tuple of two visualization_msgs/MarkerArray, MarkerArray) tuple.
            The first is of moving obstacles, and the latter is of stationary
            obstacles.
        """
        # Generate base header.
        header = Header()
        header.stamp = rospy.get_rostime()
        header.frame_id = frame

        # Parse moving obstacles, and populate markers with spheres.
        moving_obstacles = MarkerArray()
        if "moving_obstacles" in json:
            for obj in json["moving_obstacles"]:
                # Moving obstacles are spheres.
                marker = Marker()
                marker.header = header
                marker.type = Marker.SPHERE
                marker.color = cls.OBSTACLE_COLOR
                marker.ns = "moving_obstacles"
                marker.lifetime = rospy.Duration(lifetime)

                # Set scale as radius.
                radius = feet_to_meters(obj["sphere_radius"])
                marker.scale.x = marker.scale.y = marker.scale.z = radius

                # Convert latitude and longitude to UTM.
                easting, northing, _, _ = utm.from_latlon(obj["latitude"],
                                                          obj["longitude"])
                marker.pose.position.x = easting
                marker.pose.position.y = northing
                marker.pose.position.z = feet_to_meters(obj["altitude_msl"])

                moving_obstacles.markers.append(marker)

        # Parse stationary obstacles, and populate markers with cylinders.
        stationary_obstacles = MarkerArray()
        if "stationary_obstacles" in json:
            for obj in json["stationary_obstacles"]:
                # Stationary obstacles are cylinders.
                marker = Marker()
                marker.header = header
                marker.type = Marker.CYLINDER
                marker.color = cls.OBSTACLE_COLOR
                marker.ns = "stationary_obstacles"
                marker.lifetime = rospy.Duration(lifetime)

                # Set scale to define size.
                radius = feet_to_meters(obj["cylinder_radius"])
                height = feet_to_meters(obj["cylinder_height"])
                marker.scale.x = marker.scale.y = radius
                marker.scale.z = height

                # Convert latitude and longitude to UTM.
                easting, northing, _, _ = utm.from_latlon(obj["latitude"],
                                                          obj["longitude"])
                marker.pose.position.x = easting
                marker.pose.position.y = northing
                marker.pose.position.z = height / 2

                stationary_obstacles.markers.append(marker)

        return (moving_obstacles, stationary_obstacles)


class TelemetrySerializer(object):

    """Telemetry message serializer."""

    @classmethod
    def from_msg(cls, navsat_msg, compass_msg):
        """Serializes telemetry data into a dictionary.

        Args:
            navsat_msg: sensor_msgs/NavSatFix message.
            compass_msg: std_msgs/Float64 message in degrees.

        Returns:
            JSON dictionary.
        """
        return {
            "latitude": float(navsat_msg.latitude),
            "longitude": float(navsat_msg.longitude),
            "altitude_msl": meters_to_feet(navsat_msg.altitude),
            "uas_heading": float(compass_msg.data)
        }


class TargetSerializer(object):

    """Target message serializer."""

    # Enumeration message types.
    ENUMERATION_TYPES = {
        Color,
        Orientation,
        Shape,
        TargetType
    }

    @classmethod
    def from_msg(cls, msg):
        """Serializes target data into a dictionary.

        Args:
            msg: Target ROS message.

        Returns:
            JSON dictionary.
        """
        json = {}

        # Set all fields.
        for attribute in msg.__slots__:
            value = getattr(msg, attribute)
            if type(value) in cls.ENUMERATION_TYPES:
                value = value.data

            json[attribute] = value

        return json

    @classmethod
    def from_json(cls, json):
        """Deserializes target data into Target ROS message.

        Args:
            json: JSON dictionary.

        Returns:
            A Target ROS message.
        """
        # Determine ROS message type.
        msg = Target()

        for attribute in msg.__slots__:
            if attribute in json:
                # Get corresponding type of slot.
                attribute_type = type(getattr(msg, attribute))

                # Set 'casted' value.
                value = json[attribute]
                if value is not None:
                    setattr(msg, attribute, attribute_type(value))

        return msg


class TargetImageSerializer(object):

    """Target image message serializer."""

    @classmethod
    def from_msg(cls, msg):
        """Serializes a ROS Image message into a compressed PNG image.

        Args:
            msg: ROS Image message.

        Returns:
            Compressed PNG image.

        Raises:
            CvBridgeError: On image conversion error.
        """
        # Convert ROS Image to OpenCV image.
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg)

        # Convert to PNG with highest level of compression to limit bandwidth
        # usage. PNG is used since it is a lossless format, so this can later
        # be retrieved as a ROS image without issue.
        compression = [cv2.IMWRITE_PNG_COMPRESSION, 9]
        png = cv2.imencode(".png", img, compression)[1].tostring()

        return png

    @classmethod
    def from_raw(cls, raw):
        """Deserializes binary-encoded image data into a ROS Image message.

        Args:
            raw: Binary encoded image data.

        Returns:
            ROS Image message.

        Raises:
            CvBridgeError: On image conversion error.
        """
        # Convert to OpenCV image.
        nparr = np.fromstring(raw, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        # Convert to ROS message.
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(img)

        return msg
