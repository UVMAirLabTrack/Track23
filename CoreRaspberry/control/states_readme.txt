To set the light states, the text file uses 5 integers. 1 2 3 4 5


The first 4 control the color of the light 

Train control uses the same structure, however the first 4 numbers are angles from 0 to 180 degrees, corresponding to the desired angle of barrier.

the fifth is the light timing. or how long that state remains active, in seconds

  case 0:
  color = Off;
  .
  case 1:
  color = Red;
  .
    case 2:
  color = Yellow;
  .
    case 3:
  color = Green;
  .
    case 4:
  color = White;
  .
    case 5:
  color = Blue;
  .
    case 6:
  color = State6;
  .
    case 7:
  color = State7;
  .
    case 8:
  color = State8;
  .
    case 9:
  color = State9;
  .
    case 10:
  color = State10;
  .