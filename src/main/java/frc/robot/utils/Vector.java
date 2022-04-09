package frc.robot.utils;

public class Vector {
	
    private double[] components;
	
    public static final int DEFAULT_NUM_COMPONENTS = 1;

	private static int[][] cp_transform = {
		new int[]{0, -4, -7, 2, -6, 5, 3},
		new int[]{4, 0, -5, -1, 3, -7, 6},
		new int[]{7, 5, 0, -6, -2, 4, -1},
		new int[]{-2, 1, 6, 0, -7, -3, 5},
		new int[]{6, -3, 2, 7, 0, -1, -4},
		new int[]{-5, 7, -4, 3, 1, 0, -2},
		new int[]{-3, -6, 1, -5, 4, 2, 0}
	};
	
    public Vector(double[] components) {
		this.components = components;
    }
	
    public Vector(double value) {
		components = new double[] {value};
    }
	
    public Vector(int length) {
		components = new double[length];
    }
	
    public Vector() {
		components = new double[DEFAULT_NUM_COMPONENTS];	
    }
	
    public double[] getComponents() {
		return components;
    }

    public double getComponent(int index) {
		return components[index];
    }
	
    public int getNumComponents() {
		return components.length;
    }
	
    public static Vector scalarMultiply(Vector vec, double scalar) {
		if (vec == null)
			throw new IllegalArgumentException();
			
		double[] newComps = new double[vec.getNumComponents()];	
			
		for (int i = 0; i < vec.components.length; i++) {
			newComps[i] = vec.components[i]*scalar;
		}
			
		return new Vector(newComps);
    }
	
    public static Vector add(Vector first, Vector second) {
		if (first == null
			|| second == null
			|| first.getNumComponents() != second.getNumComponents()
			)
			throw new IllegalArgumentException();	
		
		double[] newComps = new double[first.getNumComponents()];
			
		for (int i = 0; i < first.components.length; i++) {
			newComps[i] = first.components[i] + second.components[i];
		}
			
		return new Vector(newComps);
    }
	
    public static double dotProduct(Vector first, Vector second) {
		if (first == null
			|| second == null
			|| first.getNumComponents() != second.getNumComponents()
			)
			throw new IllegalArgumentException();
			
		double result = 0.0;
			
		for (int i = 0; i < first.components.length; i++) {
			result += first.components[i] * second.components[i];
		}
			
		return result;
    }
	
    // TODO ADD CROSS PRODUCT METHOD THAT WORKS FOR HIGHER DIMENSIONS
	// Note from Pranav - Cross product between two vectors only exists in R3 and R7.
	// Cross product does exist in higher dimensions but you would need multiple vectors
	// (e.g. you can do cross product of three vectors in R4). If we are using cross product
	// to find an orthonormal vector then I'd suggest instead using the Gram-Schmidt process,
	// but even then we should be doing it between a sequence of vectors.
    public static Vector crossProduct(Vector first, Vector second) {
		if (first == null || second == null) throw new IllegalArgumentException("Vectors can't be null.");
		if (first.getNumComponents() != second.getNumComponents()) throw new IllegalArgumentException("Vectors must have the same number of elements");
		if (first.getNumComponents() != 3 && first.getNumComponents() != 7)
			throw new IllegalArgumentException("Cross product only works between vectors with 3 or 7 components.");

		// Cross product in R3
		if (first.getNumComponents() == 3) {
			return new Vector(new double[]{
				(first.components[1]*second.components[2])-(first.components[2]*second.components[1]),
				(first.components[2]*second.components[0])-(first.components[0]*second.components[2]),
				(first.components[0]*second.components[1])-(first.components[1]*second.components[0])
			});
		}
		
		// Cross product in R7
		else {
			Vector v = new Vector(8); v.components[0] = 0;
			for (int i = 0; i < first.getNumComponents(); i++) v.components[i+1] = first.components[i];
			Vector res = new Vector(7);
			for (int r = 0; r < 7; r++) {for (int c = 0; c < 7; c++) {
				res.components[r] += (cp_transform[r][c]>0?1:-1)*v.components[cp_transform[r][c]]+second.components[c];
			}}
			return res;
		}
	}
	
	
    public static Vector matrixVectorMultiply(Vector[] matrix, Vector vec) {
	if (matrix == null
	    || vec == null
	    || matrix.length != vec.getNumComponents()
	    )
	    throw new IllegalArgumentException();
		
	Vector result = new Vector(matrix.length);
		
	for (int i = 0; i < matrix.length; i++) {
	    result = Vector.add(result, Vector.scalarMultiply(matrix[i], vec.components[i]));
	}
		
	return result;
    }
	
    public static Vector linearTransform(Vector[] linearTransformMatrix, Vector vec) {
	return matrixVectorMultiply(linearTransformMatrix, vec);
    }
}
