package frc.robot;

public enum DeviceId {
    LifterMotor(1);

    private int id;

    private DeviceId(int id) {
        this.id = id;
    }

    public int getId() {
        return id;
    }
}