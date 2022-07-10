package frc.robot.lib.team1690;

public final class Vector3D {

    public final float x;
    public final float y;
    public final float z;

    public Vector3D(final float x, final float y, final float z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public static Vector3D fromSizeYawPitch(final float size, final float yaw, final float pitch) {
        final float vectorX = (float) (Math.cos(yaw) * Math.cos(pitch)) * size;
        final float vectorY = (float) (Math.sin(yaw) * Math.cos(pitch)) * size;
        final float vectorZ = (float) Math.sin(pitch) * size;
        return new Vector3D(vectorX, vectorY, vectorZ);
    }

    public static Vector3D zero() {
        return new Vector3D(0, 0, 0);
    }

    public Vector3D add(final Vector3D other) {
        return new Vector3D(x + other.x, y + other.y, z + other.z);
    }

    public Vector3D add(final Vector other) {
        return new Vector3D(x + other.x, y + other.y, z);
    }

    public Vector3D subtract(final Vector3D other) {
        return new Vector3D(x - other.x, y - other.y, z - other.z);
    }

    public Vector3D scale(final float factor) {
        return new Vector3D(x * factor, y * factor, z * factor);
    }

    public float norm() {
        return (float) (Math.sqrt(x * x + y * y + z * z));
    }

    public float getYaw() {
        return xy().getAngle();
    }

    public float getPitch() {
        return (float) (Math.atan2(z, xy().norm()));
    }

    public Vector xy() {
        return new Vector(x, y);
    }
}