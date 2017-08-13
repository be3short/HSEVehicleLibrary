package edu.ucsc.cross.hse.model.vehicle.navigation;

import java.util.ArrayList;

import edu.ucsc.cross.hse.core.framework.data.Data;
import edu.ucsc.cross.hse.model.position.general.Position;
import edu.ucsc.cross.hse.model.position.general.PositionData;

public class SimpleNavigationController extends WaypointConroller implements DestinationControl
{

	Data<Double> travelHeight;

	/*
	 * Constructor with thresholds defined and potentially including waypoints
	 */
	public SimpleNavigationController(Double travel_height, Double horiz_prox_threshold, Double vert_prox_threshold)
	{
		super(horiz_prox_threshold, vert_prox_threshold);
		instantiateElements(travel_height);
	}

	/*
	 * Constructor without thresholds or waypoints defined
	 */
	public SimpleNavigationController()
	{
		super();
		instantiateElements(0.0);
	}

	/*
	 * Instantiates state and data elements
	 */
	private void instantiateElements(Double travel_height)
	{
		travelHeight = new Data<Double>("Travel Height Above Ground", travel_height);
	}

	@Override
	public void updateDestination(Position position)
	{
		super.clearWaypoints();
		PositionData approachPosition = new PositionData(position.getXPosition(), position.getYPosition(),
		travelHeight.getValue());
		PositionData destinationPosition = new PositionData(position.getXPosition(), position.getYPosition(),
		position.getZPosition());
		super.addWaypoints(approachPosition, destinationPosition);
	}

}
