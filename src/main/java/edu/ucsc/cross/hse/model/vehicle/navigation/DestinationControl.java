package edu.ucsc.cross.hse.model.vehicle.navigation;

import edu.ucsc.cross.hse.core.framework.component.Component;
import edu.ucsc.cross.hse.model.position.general.Position;

public interface DestinationControl
{

	public void updateDestination(Position position);
}
