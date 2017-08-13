package edu.ucsc.cross.hse.models.app;

import edu.ucsc.cross.hse.core.framework.annotations.LibraryDefinition;
import edu.ucsc.cross.hse.model.position.euclidean.EuclideanPositionData;
import edu.ucsc.cross.hse.model.position.euclidean.EuclideanPositionStateData;
import edu.ucsc.cross.hse.model.vehicle.pointmass.PointMassVehicleSystem;
import edu.ucsc.cross.hse.model.vehicle.pointmass.SimplePointMassVehicleParameters;
import edu.ucsc.cross.hse.model.vehicle.pointmass.SimplePointMassVehicleWaypointController;

public class TestSystem
{

	@LibraryDefinition(label = "Square Path Unitary Speed Point Mass Vehicle System")
	public static PointMassVehicleSystem getSimplePointMassVehicleSystem()
	{
		EuclideanPositionStateData position = new EuclideanPositionStateData();
		SimplePointMassVehicleParameters parameters = new SimplePointMassVehicleParameters(1.0, 1.0);
		EuclideanPositionData[] waypoints = new EuclideanPositionData[]
		{ new EuclideanPositionData(0.0, 0.0, 10.0), new EuclideanPositionData(10.0, 10.0, 10.0),
				new EuclideanPositionData(10.0, -10.0, 10.0), new EuclideanPositionData(-10.0, -10.0, 10.0),
				new EuclideanPositionData(-10.0, 10.0, 10.0), new EuclideanPositionData(0.0, 0.0, 10.0),
				new EuclideanPositionData(0.0, 0.0, 0.0) };
		SimplePointMassVehicleWaypointController controller = new SimplePointMassVehicleWaypointController(.1, .1,
		waypoints);
		PointMassVehicleSystem system = new PointMassVehicleSystem(position, parameters, controller);
		return system;
	}
}
