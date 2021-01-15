package org.firstinspires.ftc.teamcode;

public class ButtonToggle {
    public boolean toggleState = false;
    private boolean buttonIsActive = false;

    boolean toggled(boolean buttonPressed) { // button is pressed = true

        if(buttonPressed && (buttonIsActive == false)) {
            buttonIsActive = true;
            toggleState = !toggleState; // true
            return true;
        }
        else if(!buttonPressed) {
            buttonIsActive = false;
        }
        return false;
    }
}