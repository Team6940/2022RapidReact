package frc.robot.lib.team3476.net.editing;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;

import java.util.function.Function;

public class LiveEditableValue<T> {
    private volatile T value;
    private final  NetworkTableEntry entry;

    private final  Function<T, Object> onWrite;

    /**
     * @param defaultValue The default value to use if the entry is not changed / the initial value set to the table
     * @param entry        The entry to listen to
     * @param onNTChange   The function to call when the entry is changed through the network table to convert the value to {@link
     *                     T}
     * @param onWrite      The function to convert to a NT object when {@link #set(T)} is called
     */
    public LiveEditableValue( T defaultValue,  NetworkTableEntry entry,  Function<Object, T> onNTChange,
                              Function<T, Object> onWrite) {
        this.onWrite = onWrite;
        this.value = defaultValue;
        this.entry = entry;
        entry.setValue(onWrite.apply(defaultValue));
        entry.addListener(event -> value = onNTChange.apply(event.value.getValue()),
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    /**
     * Create a new {@link LiveEditableValue} with the onNTChange and onWrite to simply read and write the object to NT without
     * any conversion
     *
     * @param defaultValue The default value to use if the entry is not changed / the initial value set to the table
     * @param entry        The entry to listen to
     */
    public LiveEditableValue(T defaultValue, NetworkTableEntry entry) {
        this(defaultValue, entry, value -> (T) value, value -> value);
    }

    /**
     * @return The current value
     */
    public T get() {
        return value;
    }

    /**
     * @param value The new value to set
     */
    public void set(T value) {
        this.value = value;
        entry.setValue(onWrite.apply(value));
    }
}
