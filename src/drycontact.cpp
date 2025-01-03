/****************************************************************************
 * RATGDO HomeKit for ESP32
 * https://ratcloud.llc
 * https://github.com/PaulWieland/ratgdo
 *
 * Copyright (c) 2023-25 David A Kerr... https://github.com/dkerr64/
 * All Rights Reserved.
 * Licensed under terms of the GPL-3.0 License.
 *
 * Contributions acknowledged from
 * Thomas Hagan... https://github.com/tlhagan
 *
 */

// RATGDO project includes
#include "ratgdo.h"
#include "config.h"
#include "comms.h"
#include "drycontact.h"

// Logger tag
static const char *TAG = "ratgdo-drycontact";

static bool drycontact_setup_done = false;

void onOpenSwitchPress();
void onCloseSwitchPress();
void onOpenSwitchRelease();
void onCloseSwitchRelease();

// Define OneButton objects for open/close pins
OneButton buttonOpen(DRY_CONTACT_OPEN_PIN, true, true); // Active low, with internal pull-up
OneButton buttonClose(DRY_CONTACT_CLOSE_PIN, true, true);
bool dryContactDoorOpen = false;
bool dryContactDoorClose = false;
bool previousDryContactDoorOpen = false;
bool previousDryContactDoorClose = false;

void setup_drycontact()
{
    RINFO(TAG, "=== Setting up dry contact protocol");

    if (doorControlType == 0)
        doorControlType = userConfig->getGDOSecurityType();

    doorState = DoorState::Unknown;

    pinMode(DRY_CONTACT_OPEN_PIN, INPUT_PULLUP);
    pinMode(DRY_CONTACT_CLOSE_PIN, INPUT_PULLUP);

    // Attach OneButton handlers
    buttonOpen.attachPress(onOpenSwitchPress);
    buttonClose.attachPress(onCloseSwitchPress);
    buttonOpen.attachLongPressStop(onOpenSwitchRelease);
    buttonClose.attachLongPressStop(onCloseSwitchRelease);

    drycontact_setup_done = true;
}

void drycontact_loop()
{
    if (!drycontact_setup_done)
        return;

    // Poll OneButton objects
    buttonOpen.tick();
    buttonClose.tick();

    if (doorControlType == 3)
    {
        if (dryContactDoorOpen)
        {
            doorState = DoorState::Open;
        }

        if (dryContactDoorClose)
        {
            doorState = DoorState::Closed;
        }

        if (!dryContactDoorClose && !dryContactDoorOpen)
        {
            if (previousDryContactDoorClose)
            {
                doorState = DoorState::Opening;
            }
            else if (previousDryContactDoorOpen)
            {
                doorState = DoorState::Closing;
            }
        }

        if (previousDryContactDoorOpen != dryContactDoorOpen)
        {
            previousDryContactDoorOpen = dryContactDoorOpen;
        }

        if (previousDryContactDoorClose != dryContactDoorClose)
        {
            previousDryContactDoorClose = dryContactDoorClose;
        }
    }
    else
    {
        // Dry contacts are repurposed as optional door open/close when we
        // are using Sec+ 1.0 or Sec+ 2.0 door control type
        if (dryContactDoorOpen)
        {
            open_door();
            dryContactDoorOpen = false;
        }

        if (dryContactDoorClose)
        {

            close_door();
            dryContactDoorClose = false;
        }
    }
}

/*************************** DRY CONTACT CONTROL OF DOOR ***************************/
// Functions for sensing GDO open/closed
void onOpenSwitchPress()
{
    dryContactDoorOpen = true;
    RINFO(TAG, "Open switch pressed");
}

void onCloseSwitchPress()
{
    dryContactDoorClose = true;
    RINFO(TAG, "Close switch pressed");
}

void onOpenSwitchRelease()
{
    dryContactDoorOpen = false;
    RINFO(TAG, "Open switch released");
}

void onCloseSwitchRelease()
{
    dryContactDoorClose = false;
    RINFO(TAG, "Close switch released");
}
