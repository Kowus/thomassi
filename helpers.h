#include <Arduino.h>
#include <string.h>

enum class CommandType
{
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  NONE
};
typedef struct command_t
{
  char Command;   // 'f', 'b', 'l', 'r'
  int SubCommand; // distance in cm or angle in degrees
} Command;

Command parseCommand(const String &input)
{

  Command cmd = {'\0', 0};
  if (input.length() < 2)
    return cmd; // Invalid command

  char commandChar = toupper(input.charAt(0));
  int subCommand = input.substring(1).toInt();

  switch (commandChar)
  {
  case 'F':
    cmd.Command = 'f';
    cmd.SubCommand = subCommand;
    break;
  case 'B':
    cmd.Command = 'b';
    cmd.SubCommand = subCommand;
    break;
  case 'L':
    cmd.Command = 'l';
    cmd.SubCommand = subCommand;
    break;
  case 'R':
    cmd.Command = 'r';
    cmd.SubCommand = subCommand;
    break;
  default:
    cmd.Command = '\0'; // Unknown command
    break;
  }
  return cmd;
}