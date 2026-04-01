using UnityEngine;

/// <summary>
/// UR3e forward kinematics and numerical Jacobian using standard DH parameters.
/// All computation in the robot base frame (right-handed, Z-up).
/// </summary>
public static class UR3eKinematics
{
    // Standard DH parameters for UR3e (from Universal Robots documentation)
    //            a (m)      d (m)      alpha (rad)
    private static readonly double[] A     = {  0,       -0.24355, -0.2132,  0,        0,        0       };
    private static readonly double[] D     = {  0.15185,  0,        0,       0.13105,  0.08535,  0.0921  };
    private static readonly double[] Alpha = {  System.Math.PI / 2, 0, 0, System.Math.PI / 2, -System.Math.PI / 2, 0 };

    private const double EPSILON = 1e-6;

    /// <summary>
    /// Compute the 4x4 DH transformation matrix for one joint.
    /// Standard DH: Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
    /// </summary>
    private static void DHMatrix(double theta, double d, double a, double alpha, double[,] T)
    {
        double ct = System.Math.Cos(theta);
        double st = System.Math.Sin(theta);
        double ca = System.Math.Cos(alpha);
        double sa = System.Math.Sin(alpha);

        T[0, 0] = ct;    T[0, 1] = -st * ca;  T[0, 2] = st * sa;   T[0, 3] = a * ct;
        T[1, 0] = st;    T[1, 1] = ct * ca;   T[1, 2] = -ct * sa;  T[1, 3] = a * st;
        T[2, 0] = 0;     T[2, 1] = sa;        T[2, 2] = ca;        T[2, 3] = d;
        T[3, 0] = 0;     T[3, 1] = 0;         T[3, 2] = 0;         T[3, 3] = 1;
    }

    /// <summary>
    /// Multiply two 4x4 matrices: C = A * B
    /// </summary>
    private static void Mul4x4(double[,] A4, double[,] B4, double[,] C4)
    {
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
            {
                double s = 0;
                for (int k = 0; k < 4; k++)
                    s += A4[i, k] * B4[k, j];
                C4[i, j] = s;
            }
    }

    /// <summary>
    /// Compute the full forward kinematics T_0_6 (end-effector frame in base).
    /// </summary>
    public static void FK(double[] q, double[,] T06)
    {
        var Ti = new double[4, 4];
        var temp = new double[4, 4];

        // Start with identity
        T06[0, 0] = 1; T06[0, 1] = 0; T06[0, 2] = 0; T06[0, 3] = 0;
        T06[1, 0] = 0; T06[1, 1] = 1; T06[1, 2] = 0; T06[1, 3] = 0;
        T06[2, 0] = 0; T06[2, 1] = 0; T06[2, 2] = 1; T06[2, 3] = 0;
        T06[3, 0] = 0; T06[3, 1] = 0; T06[3, 2] = 0; T06[3, 3] = 1;

        for (int i = 0; i < 6; i++)
        {
            DHMatrix(q[i], D[i], A[i], Alpha[i], Ti);
            // temp = T06 * Ti
            Mul4x4(T06, Ti, temp);
            // Copy temp back to T06
            System.Array.Copy(temp, T06, 16);
        }
    }

    /// <summary>
    /// Get end-effector position [x, y, z] from joint angles.
    /// </summary>
    public static void GetPosition(double[] q, double[] pos)
    {
        var T = new double[4, 4];
        FK(q, T);
        pos[0] = T[0, 3];
        pos[1] = T[1, 3];
        pos[2] = T[2, 3];
    }

    /// <summary>
    /// Compute the 3x6 linear velocity Jacobian numerically (finite differences on FK position).
    /// J[row, col] = d(pos[row]) / d(q[col])
    /// </summary>
    public static void JacobianLinear(double[] q, double[,] J3x6)
    {
        var qp = new double[6];
        var posPlus = new double[3];
        var posMinus = new double[3];

        for (int j = 0; j < 6; j++)
        {
            System.Array.Copy(q, qp, 6);

            qp[j] = q[j] + EPSILON;
            GetPosition(qp, posPlus);

            qp[j] = q[j] - EPSILON;
            GetPosition(qp, posMinus);

            double inv2e = 1.0 / (2.0 * EPSILON);
            J3x6[0, j] = (posPlus[0] - posMinus[0]) * inv2e;
            J3x6[1, j] = (posPlus[1] - posMinus[1]) * inv2e;
            J3x6[2, j] = (posPlus[2] - posMinus[2]) * inv2e;
        }
    }

    /// <summary>
    /// Resolve desired linear velocity [vx, vy, vz] to joint velocities using DLS pseudoinverse.
    /// Uses 3x6 linear Jacobian — does not constrain tool orientation (3-DOF, underdetermined).
    /// Minimum-norm solution: q̇ = J^T (J J^T + λ²I)^{-1} v
    /// </summary>
    public static void ResolveLinearVelocity(double[] q, double[] vDesired, double[] qDot, double damping = 0.01)
    {
        var J = new double[3, 6];
        JacobianLinear(q, J);

        // A = J * J^T + lambda^2 * I  (3x3)
        var A3 = new double[3, 3];
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                double s = 0;
                for (int k = 0; k < 6; k++)
                    s += J[i, k] * J[j, k];
                A3[i, j] = s;
            }
            A3[i, i] += damping * damping;
        }

        // Solve A * y = vDesired  (3x3 system)
        var y = new double[3];
        Solve3x3(A3, vDesired, y);

        // q̇ = J^T * y
        for (int i = 0; i < 6; i++)
        {
            double s = 0;
            for (int j = 0; j < 3; j++)
                s += J[j, i] * y[j];
            qDot[i] = s;
        }
    }

    /// <summary>
    /// Solve 3x3 linear system Ax = b using Gaussian elimination with partial pivoting.
    /// </summary>
    private static void Solve3x3(double[,] A3, double[] b, double[] x)
    {
        // Copy A and b
        var M = new double[3, 3];
        var bc = new double[3];
        for (int i = 0; i < 3; i++)
        {
            bc[i] = b[i];
            for (int j = 0; j < 3; j++)
                M[i, j] = A3[i, j];
        }

        for (int col = 0; col < 3; col++)
        {
            int maxRow = col;
            double maxVal = System.Math.Abs(M[col, col]);
            for (int row = col + 1; row < 3; row++)
            {
                double v = System.Math.Abs(M[row, col]);
                if (v > maxVal) { maxVal = v; maxRow = row; }
            }
            if (maxRow != col)
            {
                for (int j = 0; j < 3; j++)
                { double tmp = M[col, j]; M[col, j] = M[maxRow, j]; M[maxRow, j] = tmp; }
                double tmp2 = bc[col]; bc[col] = bc[maxRow]; bc[maxRow] = tmp2;
            }
            double pivot = M[col, col];
            if (System.Math.Abs(pivot) < 1e-12) { x[col] = 0; continue; }
            for (int row = col + 1; row < 3; row++)
            {
                double factor = M[row, col] / pivot;
                for (int j = col; j < 3; j++)
                    M[row, j] -= factor * M[col, j];
                bc[row] -= factor * bc[col];
            }
        }
        for (int i = 2; i >= 0; i--)
        {
            double s = bc[i];
            for (int j = i + 1; j < 3; j++)
                s -= M[i, j] * x[j];
            x[i] = System.Math.Abs(M[i, i]) > 1e-12 ? s / M[i, i] : 0;
        }
    }

    /// <summary>
    /// Get the end-effector position as a Vector3.
    /// </summary>
    public static Vector3 GetEndEffectorPosition(double[] q)
    {
        var pos = new double[3];
        GetPosition(q, pos);
        return new Vector3((float)pos[0], (float)pos[1], (float)pos[2]);
    }

    /// <summary>
    /// Compute linear manipulability: sqrt(det(Jl * Jl^T)) using the 3x6 linear Jacobian.
    /// </summary>
    public static double Manipulability(double[] q)
    {
        var J = new double[3, 6];
        JacobianLinear(q, J);

        // J * J^T (3x3)
        var JJT = new double[3, 3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                double s = 0;
                for (int k = 0; k < 6; k++)
                    s += J[i, k] * J[j, k];
                JJT[i, j] = s;
            }

        // 3x3 determinant
        double det = JJT[0, 0] * (JJT[1, 1] * JJT[2, 2] - JJT[1, 2] * JJT[2, 1])
                   - JJT[0, 1] * (JJT[1, 0] * JJT[2, 2] - JJT[1, 2] * JJT[2, 0])
                   + JJT[0, 2] * (JJT[1, 0] * JJT[2, 1] - JJT[1, 1] * JJT[2, 0]);

        return System.Math.Sqrt(System.Math.Abs(det));
    }
}
