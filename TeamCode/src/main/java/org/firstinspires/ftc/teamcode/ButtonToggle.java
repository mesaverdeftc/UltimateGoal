package org.firstinspires.ftc.teamcode;

public class ButtonToggle {
    public boolean toggleState = false;
    private boolean buttonIsActive = false;

    boolean toggled(boolean buttonPressed) {

        // buttonPressed = true

        if(buttonPressed && (buttonIsActive == false)) {
            buttonIsActive = true;
            toggleState = !toggleState; // toggleState = true
            return true;
        }
        else if(!buttonPressed) {
            buttonIsActive = false;
        }
        return false;
    }
}