# Helper functions for updating the GUI to make
# the main file less cluttered



import string
def command_to_string(command):
    if command == "movingToZero":
        return "Moving to default position"
    if command == "openingFridge":
        return "Opening the fridge"
    if command == "closingFridge":
        return "Closing the fridge"
    if command == "gettingBottleFromOpenFridgeAndPlacingOnTable":
        return "Getting water bottle and placing on table"
    if command == "placingOnTable":
        return "Placing water bottle on table"
    if command == "gettingBottleFromStart":
        return "Getting water bottle from fridge"
    if command == "openingMicrowave":
        return "Opening the microwave"
    if command == "closingMicrowave":
        return "Closing the microwave"
    if command == "puttingFoodInMicrowave":
        return "Putting food in the microwave"
    if command == "gettingFoodFromMicrowave":
        return "Getting food from the microwave"
    if command == "gettingBottlePlacingOnTable":
        return "Getting water bottle and placing on table"
    if command == "timedCook":
        return "Timed cook"
    if command == "turningOffMicrowave":
        return "Turning off microwave"
    if command == "turningOnMicrowave":
        return "Turning on microwave"
    return command


# put in another file by returning an array with two elements - suggestion one and suggestion two
def make_suggestion(lastAliveName, env):
    #use the last alive command to make suggestions
    suggestions = ["No suggested command at this time", ""]
   
    print("making suggestion")
    # Command at start #
    if lastAliveName == "movingToZero":
        suggestions[0] = '"Put a water bottle on the table"'
        suggestions[1] = '"Put food in the microwave"'
    # After Fridge open #
    elif lastAliveName == "openingFridge":
        if not env['bottleOnTable'] and not env['foodInMicrowave']:
            suggestions[0] = '"Put a water bottle on the table"'
            suggestions[1] = '"Put food in the microwave"'
        elif not env['bottleOnTable'] and env['foodInMicrowave']:
            suggestions[0] = '"Put water bottle on the table"'
            suggestions[1] = '"Get a water bottle"'
        elif env['bottleOnTable'] and not env['foodInMicrowave']:
            suggestions[0] = '"Put food in the microwave"'
            suggestions[1] = '"Close the fridge"'
        else:
            suggestions[0] = '"close the fridge"'
    # After getting bottle #
    elif (lastAliveName == "gettingBottleFromStart"
        or lastAliveName == "gettingBottleFromOpenFridge"):
        if env['hasBottle']:
            suggestions[0] = '"Place on table"'
        elif not env['hasBottle'] and env['fridgeOpen']:
            suggestions[0] = '"Close the fridge"'
    # After close fridge #
    elif lastAliveName == "closingFridge":
        if env['bottleOnTable'] and not env['foodInMicrowave']:
            suggestions[0] = '"Put food in the microwave"'
            suggestions[1] = '"Open the fridge"'
        elif not env['bottleOnTable'] and not env['foodInMicrowave']:
            suggestions[0] = '"Put water bottle on the table"'
            suggestions[1] = '"Put food in the microwave'
        elif env['foodInMicrowave'] and not env['bottleOnTable']:
            suggestions[0] = '"Put water bottle on the table"'
            suggestions[1] = '"Open the fridge"'
        #add conditions for if food is in the microwave and what to do
        else:
            suggestions[0] = '"Open the fridge"'
    # After full water bottle command #
    elif lastAliveName == "gettingBottleFromOpenFridgeAndPlacingOnTable" or "gettingBottlePlacingOnTable":
        if not env['foodInMicrowave']:
            suggestions[0] = '"Put food in the microwave"'
            suggestions[1] = '"Open the fridge"'
        else:
            suggestions[0] = '"Open the fridge"'
    # After placing bottle on table
    elif lastAliveName == "placingOnTable":
        if env['fridgeOpen']:
            suggestions[0] = '"Close the fridge"'
        else:
            if env['foodInMicrowave']:
                suggestions[0] = '"Open the fridge"'
            else:
                suggestions[0] = "'Put food in the microwave"
                suggestions[1] = "'Open the fridge'"
        if not env['foodInMicrowave']:
            suggestions[1] = '"Put food in microwave"'
    # suggestions after/during microwave commands #
    elif lastAliveName == "openingMicrowave":
        if env['foodInMicrowave']:
            suggestions[0] = '"Get food from the microwave"'
            suggestions[1] = '"Close the microwave"'
        else:
            suggestions[0] = '"Put food in the microwave"'
            suggestions[1] = '"Close the microwave"'
    elif lastAliveName == "closingMicrowave":
        if env['foodInMicrowave']:
            suggestions[0] = '"Cook for ___ seconds"'
            suggestions[1] = '"Get food from the microwave"'
        else: # MAKE ENV BOOLEAN FOR FOOD LOCATION #
            suggestions[0] = '"Put food in the microwave"'
            if not env['bottleOnTable']:
                suggestions[1] = '"Put a water bottle on the table"'
            else:
                if env['fridgeOpen']:
                    suggestions[1] = '"Close the fridge"'
                else:
                    suggestions[1] = '"Open the fridge"'
    elif lastAliveName == "turningOnMicrowave":
        suggestions[0] = '"Turn off"'
    elif lastAliveName ==  "turningOffMicrowave":
        if env['foodInMicrowave']:
            suggestions[0] = '"Get food from microwave"'
            suggestions[1] = '"Cook for ____ seconds"'
        else:
            suggestions[0] = '"Put food in the microwave"'
            suggestions[1] = '"Open the microwave"'
    elif lastAliveName == "puttingFoodInMicrowave":
        suggestions[0] = '"Cook for ____ seconds"'
        suggestions[1] = '"Turn on microwave"'
    elif lastAliveName == "gettingFoodFromMicrowave":
        if env['microwaveOpen']:
            suggestions[0] = '"Close the microwave"'
        else:
            if not env['bottleOnTable']:
                suggestions[0] = '"Put water bottle on table"'
                suggestions[1] = '"Open the fridge"'

    return suggestions