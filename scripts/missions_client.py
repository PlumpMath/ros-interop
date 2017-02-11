#!/usr/bin/env python

"""Mission Information Client"""

from threading import Lock

import rospy
from interop import InteroperabilityClient
from simplejson import JSONDecodeError
from requests.exceptions import Timeout, ConnectionError, HTTPError
from geometry_msgs.msg import PointStamped, PolygonStamped
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
from interop.srv import GetMissionByID
from interop.msg import FlyZoneArray


def publish_mission(timer):
    """Requests and publishes the mission information."""
    with lock:
        flyzones_pub.publish(msgs[0])
        search_grid_pub.publish(msgs[1])
        waypoints_pub.publish(msgs[2])
        air_drop_pub.publish(msgs[3])
        off_axis_targ_pub.publish(msgs[4])
        emergent_targ_pub.publish(msgs[5])
        utm_zone_pub.publish(msgs[6])

def get_active_mission(req):
    """ Service to update mission information with current active mission.

    Args:
        req: Request of type Trigger.

    Returns:
        TriggerResponse with true, false for success, failure.
    """
    with lock:
        global msgs
        try:
            msgs = client.get_active_mission(frame)
        except (ConnectionError, Timeout) as e:
            rospy.logwarn(e)
            return False, str(e)
        except (JSONDecodeError, HTTPError) as e:
            rospy.logerr(e)
            return False, str(e)

    rospy.loginfo("Using active mission")
    return True, "Success"


def get_mission_by_id(req):
    """ Service to update mission information with specific mission as given
    by id.

    Args:
        req: GetMissionByID type request with field id corresponding to the
             mission

    Returns:
        GetMissionByIdResponse which is true for success and false for
        failure.
    """
    with lock:
        global msgs
        try:
            msgs = client.get_mission(req.id, frame)
        except (ConnectionError, Timeout) as e:
            rospy.logwarn(e)
            return False, str(e)
        except (JSONDecodeError, HTTPError) as e:
            rospy.logerr(e)
            return False, str(e)

    rospy.loginfo("Using mission ID: %d", req.id)
    return True, "Success"


if __name__ == "__main__":
    rospy.init_node("mission_info")

    # Get server login information.
    url = rospy.get_param("~base_url")
    username = rospy.get_param("~username")
    password = rospy.get_param("~password")
    timeout = rospy.get_param("~timeout")
    client = InteroperabilityClient(url, username, password, timeout)

    # Login.
    client.wait_for_server()
    client.login()

    # Get topics to publish to.
    flyzones_topic = rospy.get_param("~flyzones_topic")
    search_grid_topic = rospy.get_param("~search_grid_topic")
    waypoints_topic = rospy.get_param("~waypoints_topic")
    air_drop_topic = rospy.get_param("~air_drop_loc_topic")
    off_axis_targ_topic = rospy.get_param("~off_axis_targ_topic")
    emergent_targ_topic = rospy.get_param("~emergent_targ_topic")
    utm_zone_topic = rospy.get_param("~utm_zone_topic")

    # Setup publishers.
    flyzones_pub = rospy.Publisher(flyzones_topic, FlyZoneArray,
                                   queue_size=1)
    search_grid_pub = rospy.Publisher(search_grid_topic, PolygonStamped,
                                      queue_size=1)
    waypoints_pub = rospy.Publisher(waypoints_topic, Marker, queue_size=1)
    air_drop_pub = rospy.Publisher(air_drop_topic, PointStamped,
                                   queue_size=1)
    off_axis_targ_pub = rospy.Publisher(off_axis_targ_topic, PointStamped,
                                        queue_size=1)
    emergent_targ_pub = rospy.Publisher(emergent_targ_topic, PointStamped,
                                       queue_size=1)
    utm_zone_pub = rospy.Publisher(utm_zone_topic, String, queue_size=1)

    # Get message parameters.
    frame = str(rospy.get_param("~frame"))

    # Create timer event and publish.
    period = float(rospy.get_param("~period"))

    # Create lock for msgs.
    lock = Lock()

    # Setup services and publish mission.
    rospy.Service("get_active_mission", Trigger, get_active_mission)
    rospy.Service("get_mission_by_id", GetMissionByID, get_mission_by_id)

    # Get mission to begin publishing. This is the first mission published.
    mission_id = rospy.get_param("~id")
    msgs = None

    retry_rate = rospy.Rate(1)
    while msgs is None and not rospy.is_shutdown():
        # If id is negative then it is default and non existent.
        if mission_id >= 0:
            req = GetMissionByID()
            req.id = mission_id
            get_mission_by_id(req)
        else:
            get_active_mission(Trigger())

        retry_rate.sleep()

    # Publish message on timer.
    timer = rospy.Timer(rospy.Duration(period), publish_mission)

    rospy.spin()
