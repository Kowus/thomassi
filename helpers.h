#include <Arduino.h>
#include <string.h>

int ack = 1; // Acknowledge variable for command processing
enum class CommandType
{
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  NONE,
  ERROR // Used for unknown commands
};

typedef struct command_t
{
  char Command; // 'f', 'b', 'l', 'r'
  CommandType desiredAction;
  int SubCommand; // distance in cm or angle in degrees
} Command;

// CommandType desiredAction = CommandType::NONE;

Command parseCommand(const String &input)
{

  Command cmd = {'\0', CommandType::NONE, 0};
  if (input.length() < 2)
    return cmd; // Invalid command

  char commandChar = toupper(input.charAt(0));
  int subCommand = input.substring(1).toInt();

  switch (commandChar)
  {
  case 'F':
    cmd.Command = 'f';
    cmd.desiredAction = CommandType::FORWARD;
    cmd.SubCommand = subCommand;
    break;
  case 'B':
    cmd.Command = 'b';
    cmd.desiredAction = CommandType::BACKWARD;
    cmd.SubCommand = subCommand;
    break;
  case 'L':
    cmd.Command = 'l';
    cmd.desiredAction = CommandType::LEFT;
    cmd.SubCommand = subCommand;
    break;
  case 'R':
    cmd.Command = 'r';
    cmd.desiredAction = CommandType::RIGHT;
    cmd.SubCommand = subCommand;
    break;
  default:
    cmd.Command = '\0'; // Unknown command
    cmd.SubCommand = 0; // Reset subcommand
    ack = 1;            // Set error code for unknown command
    break;
  }
  return cmd;
}