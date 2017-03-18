# -*- coding: utf-8 -*-

"""Interoperability HTTP Client."""

import json
import rospy
import requests
import serializers


class InteroperabilityClient(object):

    """InteroperabilityClient.

    Attributes:
        url: Base URL of the Interoperability server.
        session: Requests session.
        timeout: Timeout in seconds for individual requests.
    """

    def __init__(self, url, username, password, timeout):
        """Initializes InteroperabilityClient.

        Note: the client must wait_for_server() and login() to the server
        before first use.

        Args:
            url: Interoperability server base URL (e.g. http://127.0.0.1:8080).
            username: Interoperability server username.
            password: Interoperability server password.
            timeout: Timeout in seconds for individual requests.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
        """
        self.timeout = timeout
        self.url = url[:-1] if url.endswith('/') else url
        self.session = requests.Session()

        # Set up credentials for login.
        self.__credentials = {"username": username, "password": password}

    def __request(self, method, uri, **kwargs):
        """Sends request to Interoperability server at specified URI.

        Args:
            method: HTTP method.
            uri: Server URI to access.
            **kwargs: Arguments to HTTP Request.

        Returns:
            HTTP Response.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        # Try until authenticated.
        while not rospy.is_shutdown():
            # Send request.
            response = self.session.request(
                method=method,
                url=self.url + (uri if uri.startswith('/') else '/' + uri),
                timeout=self.timeout,
                **kwargs)

            # Relogin if session expired, and try again.
            if response.status_code == requests.codes.FORBIDDEN:
                rospy.logwarn("Session expired: reauthenticating...")

                # Start a new session.
                self.session.close()
                self.session = requests.Session()

                # Relogin.
                self.login()
                continue

            # Notify of other errors.
            response.raise_for_status()

            # All is good.
            break

        return response

    def _get(self, uri, **kwargs):
        """Sends GET request to Interoperability server at specified URI.

        Args:
            uri: Server URI to access.
            **kwargs: Arguments to HTTP Request.

        Returns:
            HTTP Response.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        return self.__request("GET", uri, **kwargs)

    def _put(self, uri, **kwargs):
        """Sends PUT request to Interoperability server at specified URI.

        Args:
            uri: Server URI to access.
            **kwargs: Arguments to HTTP Request.

        Returns:
            HTTP Response.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        return self.__request("PUT", uri, **kwargs)

    def _post(self, uri, **kwargs):
        """Sends POST request to Interoperability server at specified URI.

        Args:
            uri: Server URI to access.
            **kwargs: Arguments to HTTP Request.

        Returns:
            HTTP Response.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        return self.__request("POST", uri, **kwargs)

    def _delete(self, uri, **kwargs):
        """Sends DELETE request to Interoperability server at specified URI.

        Args:
            uri: Server URI to access.
            **kwargs: Arguments to HTTP Request.

        Returns:
            HTTP Response.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        return self.__request("DELETE", uri, **kwargs)

    def wait_for_server(self):
        """Waits until interoperability server is reachable."""
        reachable = False
        while not reachable and not rospy.is_shutdown():
            try:
                response = requests.get(self.url, timeout=self.timeout)
                response.raise_for_status()
                reachable = response.ok
            except:
                continue

    def login(self):
        """Authenticates with the server.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        response = self.session.request(
            method="POST",
            url=self.url + "/api/login",
            timeout=self.timeout,
            data=self.__credentials)
        response.raise_for_status()

    def get_obstacles(self, frame, lifetime):
        """Returns obstacles as Markers.

        Args:
            frame: Frame ID of every Marker.
            lifetime: Lifetime of every Marker in seconds.

        Returns:
            Tuple of two visualization_msgs/MarkerArray.
            The first is of moving obstacles, and the latter is of stationary
            obstacles.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            JSONDecodeError: On JSON decoding failure.
        """
        response = self._get("/api/obstacles")
        return serializers.ObstaclesDeserializer.from_json(response.json(),
                                                           frame, lifetime)

    def post_telemetry(self, navsat_msg, compass_msg):
        """Uploads telemetry information to Interoperability server.

        Args:
            navsat_msg: sensor_msgs/NavSatFix message.
            compass_msg: std_msgs/Float64 message in degrees.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        json_telem = serializers.TelemetrySerializer.from_msg(navsat_msg,
                                                              compass_msg)
        self._post("/api/telemetry", data=json.dumps(json_telem))

    def post_target(self, target):
        """Uploads new target for submission.

        Args:
            target: Target ROS message.

        Returns:
            Target ID.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            JSONDecodeError: On JSON decoding failure.
        """
        json_target = serializers.TargetSerializer.from_msg(target)
        response = self._post("/api/targets", data=json.dumps(json_target))
        return response.json()["id"]

    def get_all_targets(self):
        """Returns first 100 submitted targets.

        Returns:
            Dictionary of Target IDs to Target ROS messages.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            JSONDecodeError: On JSON decoding failure.
        """
        response = self._get("/api/targets")
        targets = {
            t["id"]: serializers.TargetSerializer.from_json(t)
            for t in response.json()
        }
        return targets

    def get_target(self, id):
        """Returns target with matching ID.

        Args:
            id: Target ID.

        Returns:
            A Target ROS message.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            JSONDecodeError: On JSON decoding failure.
        """
        response = self._get("/api/targets/{:d}".format(id))
        target = serializers.TargetSerializer.from_json(response.json())
        return target
	
    def get_active_mission(self, frame):
        """Gets active mission.

        Args:
            frame: Frame ID.

        Returns
            A tuple of (FlyZoneArray, PolygonStamped, Marker, PointStamped, 
            PointStamped, Pointstamped, String) corresponding to the flyzones, 
            search grid, waypoints, air drop position, off axis target location, 
            the emergent object location, and the utm zone of the active 
            mission.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            JSONDecodeError: On JSON decoding failure.
            LookupError: On no active missions found.
        """	
        response = self._get("/api/missions")
        for m in response.json():
            if m["active"]:
                return serializers.MissionDeserializer.from_json(m, frame)
        raise LookupError("No active missions found")

    def get_all_missions(self, frame):
        """Gets all missions.

        Args: 
            frame: Frame ID.

        Returns:
            A list of tuples of (FlyZoneArray, PolygonStamped, Marker, 
            PointStamped, PointStamped, Pointstamped, String) corresponding to 
            the flyzones, search grid, waypoints, air drop position,
            off axis target location, the emergent object location, and the
            UTM Zone.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            JSONDecodeError: On JSON decoding failure.
        """
        response = self._get("/api/missions")
        missions = {
            m["id"]: serializers.MissionDeserializer.from_json(m, frame)
            for m in response.json()
        }
        return missions

    def get_mission(self, id, frame):
        """Returns mission with the matching ID.
           
        Args:
            id: Mission ID.
            frame: Frame ID.

        Returns:
            A tuple of (FlyZoneArray, PolygonStamped, Marker, PointStamped, 
            PointStamped, Pointstamped, String) corresponding to the flyzones, 
            search grid, waypoints, air drop position, off axis target location,
            the emergent object location, and the UTM zone of the corresponding
            mission.

        Raises: 
            Timeout: On Timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            JSONDecodeError: On JSON decoding failure.            
        """
        response = self._get("/api/missions/{:d}".format(id))
        mission = serializers.MissionDeserializer.from_json(response.json(), frame)
        return mission

    def put_target(self, id, target):
        """Updates target information.

        Args:
            id: Target ID.
            target: Target ROS message.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        json_target = serializers.TargetSerializer.from_msg(target)
        self._put("/api/targets/{:d}".format(id), data=json.dumps(json_target))

    def delete_target(self, id):
        """Deletes target with matching ID.

        Args:
            id: Target ID.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        self._delete("/api/targets/{:d}".format(id))

    def post_target_image(self, id, img):
        """Adds or updates target image thumbnail as a compressed PNG.

        Args:
            id: Target ID.
            img: Target ROS message.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            CvBridgeError: On image conversion failure.
        """
        png = serializers.TargetImageSerializer.from_msg(img)
        self._post("/api/targets/{:d}/image".format(id), data=png)

    def get_target_image(self, id):
        """Retrieves target image thumbnail.

        Args:
            id: Target ID.

        Returns:
            A ROS Image message.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            CvBridgeError: On image conversion failure.
        """
        response = self._get("/api/targets/{:d}/image".format(id))
        img = serializers.TargetImageSerializer.from_raw(response.content)
        return img

    def delete_target_image(self, id):
        """Deletes target image thumbnail.

        Args:
            id: Target ID.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        self._delete("/api/targets/{:d}/image".format(id))
