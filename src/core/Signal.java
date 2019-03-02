package core;

/**
 * The car's turn signal.
 */
public enum Signal {
	LEFT, NONE, RIGHT;

	/**
	 * Get the opposite signal.
	 * 
	 * @return the opposite signal
	 */
	public Signal opposite () {
		switch ( this ) {
		case LEFT:
			return RIGHT;
		case RIGHT:
			return LEFT;
		default:
			return NONE;
		}
	}
}
