// keyboard.h
// Interface for the keyboard process (I).
// ======================================================================

#ifndef KEYBOARD_H
#define KEYBOARD_H

// Run the keyboard process:
//   - reads from stdin
//   - sends KeyMsg to B via write_fd
void run_keyboard_process(int write_fd);

#endif // KEYBOARD_H
