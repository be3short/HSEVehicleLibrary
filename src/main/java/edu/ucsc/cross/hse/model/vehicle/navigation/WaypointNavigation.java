package edu.ucsc.cross.hse.model.vehicle.navigation;

import edu.ucsc.cross.hse.model.position.general.Position;

public interface WaypointNavigation
{

	public void clearWaypoints();

	public void addWaypoints(Position... waypoints);

}
