package edu.ucsc.cross.hse.model.vehicle.pointmass;

import edu.ucsc.cross.hse.core.framework.annotations.LibraryDefinition;
import edu.ucsc.cross.hse.model.position.general.PositionData;
import edu.ucsc.cross.hse.model.position.general.PositionStateData;
import edu.ucsc.cross.hse.model.vehicle.general.Vehicle;
import edu.ucsc.cross.hse.model.vehicle.navigation.DestinationControl;

public class TestSystem
{

	@LibraryDefinition(label = "Square Path Unitary Speed Point Mass Vehicle System")
	public static PointMassVehicleSystem<DestinationControl> getSimplePointMassVehicleSystem()
	{
		PositionStateData position = new PositionStateData();
		SimplePointMassVehicleParameters parameters = new SimplePointMassVehicleParameters(1.0, 1.0);
		PositionData[] waypoints = new PositionData[]
		{ new PositionData(0.0, 0.0, 10.0), new PositionData(10.0, 10.0, 10.0), new PositionData(10.0, -10.0, 10.0),
				new PositionData(-10.0, -10.0, 10.0), new PositionData(-10.0, 10.0, 10.0),
				new PositionData(0.0, 0.0, 10.0), new PositionData(0.0, 0.0, 0.0) };
		SimplePointMassVehicleNavigationController controller = new SimplePointMassVehicleNavigationController(.1, .1,
		100.0);
		controller.addWaypoints(waypoints);

		PointMassVehicleSystem<DestinationControl> system = new PointMassVehicleSystem<DestinationControl>(position,
		parameters, controller);
		return system;
	}

	@LibraryDefinition(label = "Randomized Destination Speed Point Mass Vehicle System")
	public static PointMassVehicleSystem<DestinationControl> getSimplePointMassRandomVehicleSystem()
	{
		PositionStateData position = new PositionStateData();
		SimplePointMassVehicleParameters parameters = new SimplePointMassVehicleParameters(1.0, 1.0);
		RandomizedSimplePointMassVehicleNavigationController controller = new RandomizedSimplePointMassVehicleNavigationController();
		// dat.component().getLabels().setName("Dat");
		PointMassVehicleSystem<DestinationControl> system = new PointMassVehicleSystem<DestinationControl>(position,
		parameters, controller);

		return system;
	}

	public static Vehicle<DestinationControl> getVehicle()
	{
		return getSimplePointMassRandomVehicleSystem();
	}
}
