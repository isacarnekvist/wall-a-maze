#ifndef TEXT_COLOR_H
#define TEXT_COLOR_H

namespace textColor {
	const char white[] = { 0x1b, '[', '1', ';', '3', '7', 'm', 0 };         // White
	const char lightRed[] = { 0x1b, '[', '1', ';', '3', '1', 'm', 0 };      // Light Red
	const char green[] = { 0x1b, '[', '0', ';', '3', '2', 'm', 0 };         // Green
	const char lightGreen[] = { 0x1b, '[', '1', ';', '3', '2', 'm', 0 };	// Light Green
	const char red[] = { 0x1b, '[', '0', ';', '3', '1', 'm', 0 };              // Red
	const char yellow[] = { 0x1b, '[', '1', ';', '3', '3', 'm', 0 };        // Yellow
	const char cyan[] = { 0x1b, '[', '0', ';', '3', '6', 'm', 0 };          // Cyan
	const char lightPurple[] = { 0x1b, '[', '1', ';', '3', '5', 'm', 0 };	// Light Purple
	const char blue[] = { 0x1b, '[', '0', ';', '3', '4', 'm', 0 };          // Blue
}

#endif // TEXT_COLOR_H
