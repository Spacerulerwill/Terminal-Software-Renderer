#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <stdio.h>
#include <array>
#include <fmt/format.h>
#include <chrono>
#include <thread>
#include <numbers>
#include <math.h>
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

struct Terminal {
    unsigned int width;
    unsigned int height;
};

#if defined(__linux__) || defined(__APPLE__)

#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#define SWITCH_TO_ALT_TERMINAL "\u001B[?1049h" 
#define RESTORE_TERMINAL "\u001B[?1049l" 

// Get terminal information
Terminal getTerminal() {
    struct winsize w;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
    return Terminal{
        .width = static_cast<unsigned int>(w.ws_col),
        .height = static_cast<unsigned int>(w.ws_row),
    };
}

// Move cursor to start of terminal
void MoveCursorToStart() {
    std::cout << "\033[0;0f";
}

char getch_noblock() {
	char buf = 0;
	struct termios old {};
	if (tcgetattr(0, &old) < 0)
		perror("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	old.c_cc[VMIN] = 0;
	if (tcsetattr(0, TCSANOW, &old) < 0)
		perror("tcsetattr ICANON");
	if (read(0, &buf, 1) < 0)
		perror ("read()");
	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(0, TCSADRAIN, &old) < 0)
		perror ("tcsetattr ~ICANON");
	return buf;
}

#elif defined(_WIN32)

#define NOMINMAX
#include <Windows.h>
#include <conio.h>

Terminal getTerminal() {
    CONSOLE_SCREEN_BUFFER_INFO csbi;
    int columns, rows;
    GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &csbi);
    columns = csbi.srWindow.Right - csbi.srWindow.Left + 1;
    rows = csbi.srWindow.Bottom - csbi.srWindow.Top + 1;
    return Terminal{
        .width = static_cast<unsigned int>(columns),
        .height = static_cast<unsigned int>(rows),
    };
 }

void MoveCursorToStart() {
    static const HANDLE std_handle = GetStdHandle(STD_OUTPUT_HANDLE);
    static const COORD top_left = { 0, 0 };
    SetConsoleCursorPosition(std_handle, top_left);
}

int getch_noblock() {
    if (_kbhit())
        return _getch();
    else
        return -1;
}

#else
#error Platform not supported!
#endif

struct Pixel {
	char colorCode[19];
	char pixel;
};

const char pixelChar = '#';
const char noPixelChar = ' ';

using iVec2 = std::array<int, 2>;
using iVec3 = std::array<int, 3>;
using Vec2 = std::array<float, 2>;
using Vec3 = std::array<float, 3>;
using Vec4 = std::array<float, 4>;

struct Vertex {
	Vec3 pos;
	Vec3 color;
	Vec2 texPos;
};

// Probably only works with 8 bit png!
struct Texture {
	Texture(const char* path) {
		data = stbi_load(path, &width, &height, &numComponents, 0);
	}

	iVec3 GetPixel(iVec2 pos) const {
		unsigned char* pixelOffset = data + (pos[1] * width + pos[0]) * numComponents;
		unsigned char r = pixelOffset[0];
		unsigned char g = pixelOffset[1];
		unsigned char b = pixelOffset[2];
		return iVec3{r, g, b};
	}

	~Texture() {
		stbi_image_free(data);
	}
	stbi_uc* data = nullptr;
	int width = 0;
	int height = 0;
	int numComponents;
};

constexpr float PI_OVER_180 = static_cast<float>(std::numbers::pi) / 180.0f;
float radians(float theta)
{
    return theta * PI_OVER_180;
}

Vec2 ndsToPixelCoordinates(Vec2 nds, Terminal terminal) {
	return Vec2 {
		round((nds[0] + 1.0f) * 0.5f * static_cast<float>(terminal.width - 1)),
		round((1.0f - nds[1]) * 0.5f * static_cast<float>(terminal.height - 1))
	};
}

Vec3 rotate(Vec3 point, Vec3 center, float angle) {
	Vec3 rot;
	point[0] -= center[0];
	point[1] -= center[1];
	point[2] -= center[2];
	rot[0] = point[0] * std::cos(angle) + point[2] * std::sin(angle);
	rot[1] = point[1];
	rot[2] = point[2] * std::cos(angle) - point[0] * std::sin(angle);
	rot[0] += center[0];
	rot[1] += center[1];
	rot[2] += center[2];
	return rot;
}

std::string getColorEscapeCode(Vec3 color) {
	return fmt::format(
		"\x1b[38;2;{:03d};{:03d};{:03d}m",
		static_cast<int>(color[0] * 255),
		static_cast<int>(color[1] * 255),
		static_cast<int>(color[2] * 255)
	);
}

std::string getColorEscapeCode(iVec3 color) {
	return fmt::format(
		"\x1b[38;2;{:03d};{:03d};{:03d}m",		
		color[0],
		color[1],
		color[2]
	);
}

float edge_cross(Vec2 a, Vec2 b, Vec2 p) {
	Vec2 ab = { b[0] - a[0], b[1] - a[1] };
	Vec2 ap = { p[0] - a[0], p[1] - a[1] };
	return ab[0] * ap[1] - ab[1] * ap[0];
}

bool is_top_left(Vec2 start, Vec2 end) {
	Vec2 edge = { end[0] - start[0], end[1] - start[1] };
	bool is_top_edge = edge[1] == 0 && edge[0] > 0;
	bool is_left_edge = edge[1] < 0;
	return is_left_edge || is_top_edge;
}

Vec4 apply_perspective(Vec3 point, float fov, float aspect, float near_dist, float far_dist) 
{
	const float f = 1.0f / std::tan(fov / 2.0f);
	return Vec4 {
		point[0] * f,
		point[1] * f,
		(point[2] * ((far_dist + near_dist) / (near_dist - far_dist))) + 1.0f * ((2 * far_dist * near_dist) / (near_dist - far_dist)),
		point[2] * -1.0f 
	};
}

void writePixel(std::vector<Pixel>& pixels, iVec2 pos, std::string colorCode, Terminal terminal) {
	if (pos[0] < 0 || pos[0] >= static_cast<int>(terminal.width) || pos[1] < 0 || pos[1] >= static_cast<int>(terminal.height)) {
		return;
	}
	std::size_t pxlIdx = (terminal.width * static_cast<unsigned int>(pos[1])) + static_cast<unsigned int>(pos[0]);
	memcpy(pixels[pxlIdx].colorCode, colorCode.c_str(), 19);
	pixels[pxlIdx].pixel = pixelChar;
}

void rasterize_triangle(const Texture& texture, std::vector<Pixel>& pixels, std::vector<float>& depthBuffer, Vertex v0, Vertex v1, Vertex v2, Terminal terminal) {	
	// Multiply our world space coordinates by our projection matrix
	Vec4 v0_perspective = apply_perspective(v0.pos, radians(90.0f), static_cast<float>(terminal.width) / static_cast<float>(terminal.height), 0.1f, 100.0f);
	Vec4 v1_perspective = apply_perspective(v1.pos, radians(90.0f), static_cast<float>(terminal.width) / static_cast<float>(terminal.height), 0.1f, 100.0f);
	Vec4 v2_perspective = apply_perspective(v2.pos, radians(90.0f), static_cast<float>(terminal.width) / static_cast<float>(terminal.height), 0.1f, 100.0f);
	
	// Convert their projected positions for NDS (making sure to divide by w component)
	Vec2 pos0 = ndsToPixelCoordinates(Vec2{v0_perspective[0] / v0_perspective[3], v0_perspective[1] / v0_perspective[3]}, terminal); 
	Vec2 pos1 = ndsToPixelCoordinates(Vec2{v1_perspective[0] / v1_perspective[3], v1_perspective[1] / v1_perspective[3]}, terminal);
	Vec2 pos2 = ndsToPixelCoordinates(Vec2{v2_perspective[0] / v2_perspective[3], v2_perspective[1] / v2_perspective[3]}, terminal);

	// Find the minimum bounding box around the triangle to cheaply reduce the amount of pixels needed to check 
	int x_min = static_cast<int>(floorf(std::min(std::min(pos0[0], pos1[0]), pos2[0])));
	int y_min = static_cast<int>(floorf(std::min(std::min(pos0[1], pos1[1]), pos2[1])));
	int x_max = static_cast<int>(ceilf(std::max(std::max(pos0[0], pos1[0]), pos2[0])));
	int y_max = static_cast<int>(ceilf(std::max(std::max(pos0[1], pos1[1]), pos2[1])));
	
	// Area of the whole triangle
	float area = edge_cross(pos0, pos1, pos2);

	// Compute constant deltas that will be used to horizontal and vertical steps 
	float delta_w0_col = (pos1[1] - pos2[1]);
	float delta_w1_col = (pos2[1] - pos0[1]);
	float delta_w2_col = (pos0[1] - pos1[1]);
	float delta_w0_row = (pos2[0] - pos1[0]);
	float delta_w1_row = (pos0[0] - pos2[0]);
	float delta_w2_row = (pos1[0] - pos0[0]);

	// Rasterization fill rule, not 100% precise due to floating point innacuracy
	float bias0 = is_top_left(pos1, pos2) ? 0.0f : -0.0001f;
	float bias1 = is_top_left(pos2, pos0) ? 0.0f : -0.0001f;
	float bias2 = is_top_left(pos0, pos1) ? 0.0f : -0.0001f;

	// Edge functions for top left pixel 
	Vec2 p0 = { static_cast<float>(x_min) + 0.5f , static_cast<float>(y_min) + 0.5f };
	float w0_row = edge_cross(pos1, pos2, p0) + bias0;
	float w1_row = edge_cross(pos2, pos0, p0) + bias1;
	float w2_row = edge_cross(pos0, pos1, p0) + bias2;

	// Loop over bounding box 
	for (int y = y_min; y <=y_max; y++) {
		float w0 = w0_row;
		float w1 = w1_row;
		float w2 = w2_row;
		
		for (int x = x_min; x <= x_max; x++) {
			// Check if inside - disable one of these to only render one based on winding order
			bool is_inside = (w0 <= 0 && w1 <= 0 && w2 <= 0) || (w0 >= 0 && w1 >= 0 && w2 >= 0);
			if (is_inside) {
				// Barycentric coordinates
				float alpha = w0 / area;
				float beta  = w1 / area;
				float gamma = w2 / area;
				// Linearly interpolate w component
				float w = alpha * (1.0f / v0_perspective[3]) + beta * (1.0f / v1_perspective[3]) + gamma * (1.0f / v2_perspective[3]);	
				// Interpolate color between vertices
				Vec3 color {
					(alpha * (v0.color[0] / v0_perspective[3]) + beta * (v1.color[0] / v1_perspective[3]) + gamma * (v2.color[0] / v2_perspective[3])) / w,
					(alpha * (v0.color[1] / v0_perspective[3]) + beta * (v1.color[1] / v1_perspective[3]) + gamma * (v2.color[1] / v2_perspective[3])) / w,
					(alpha * (v0.color[2] / v0_perspective[3]) + beta * (v1.color[2] / v1_perspective[3]) + gamma * (v2.color[2] / v2_perspective[3])) / w,
				};	

				// Interpolate texture coordinates, dividing by w components
				float u = alpha * (v0.texPos[0] / v0_perspective[3]) + beta * (v1.texPos[0] / v1_perspective[3]) + gamma * (v2.texPos[0] / v2_perspective[3]); 
				float v = alpha * (v0.texPos[1] / v0_perspective[3]) + beta * (v1.texPos[1] / v1_perspective[3]) + gamma * (v2.texPos[1] / v2_perspective[3]);
				// Calculate texel pos
				iVec2 texelPos {
					static_cast<int>((u / w) * static_cast<float>(texture.width - 1)),
					static_cast<int>((v / w) * static_cast<float>(texture.height - 1))
				};
				// Sample texel from texture
				iVec3 texelColor = texture.GetPixel(texelPos);	
				// Combine pixel color and texture color
				Vec3 finalColor {
					color[0] * (static_cast<float>(texelColor[0]) / 255.0f),
					color[1] * (static_cast<float>(texelColor[1]) / 255.0f),
					color[2] * (static_cast<float>(texelColor[2]) / 255.0f)
				};
				float depth = alpha * (v0_perspective[2] / v0_perspective[3]) + beta * (v1_perspective[2] / v1_perspective[3]) + gamma * (v2_perspective[2] / v2_perspective[3]);
				std::size_t pxlIdx = (y * terminal.width) + x;
				if (depth < depthBuffer[pxlIdx]) {
					depthBuffer[pxlIdx] = depth;
					writePixel(pixels, iVec2{x, y}, getColorEscapeCode(finalColor), terminal);
				}
			}
			w0 += delta_w0_col;
			w1 += delta_w1_col;
			w2 += delta_w2_col;
		}
		w0_row += delta_w0_row;
		w1_row += delta_w1_row;
		w2_row += delta_w2_row;
	}
};


int main() {
	stbi_set_flip_vertically_on_load(1);

	// switch to alternative terminal
#ifdef __linux__
    std::cout << SWITCH_TO_ALT_TERMINAL;
#endif 
    MoveCursorToStart();
	// get terminal dimensions
    Terminal terminal = getTerminal();
	std::size_t pixelCount = terminal.width * terminal.height;
	
	// Our vertices - anticlockwise winding order
	std::array vertices = {
		// quad 1
		Vertex {
			.pos = {-0.5f, -0.5f, -1.0f},
			.color = {1.0f, 0.0f, 0.0f},
			.texPos = {0.0f, 0.0f},
		},
		Vertex {
			.pos = {0.5f, -0.5f, -1.0f},
			.color = {0.0f, 1.0f, 0.0f},
			.texPos = {1.0f, 0.0f},
		},
		Vertex {
			.pos = {-0.5f, 0.5f, -1.0f},
			.color = {0.0f, 0.0f, 1.0f},
			.texPos = {0.0f, 1.0f}
		},
		Vertex {
			.pos = {0.5f, 0.5f, -1.0f},
			.color = {1.0f, 0.0f, 0.0f},
			.texPos = {1.0f, 1.0f}
		},
	};	

	std::array<std::size_t, 6> indices {
		0,1,2,
		3,2,1
	};

	// Load texture
	Texture texture("awesomeface.png");
	auto a = std::chrono::system_clock::now();
	auto b = std::chrono::system_clock::now();
	double frameTime = (1.0 / 15) * 1000;

	std::vector<Pixel> pixels(pixelCount);
	std::vector<float> depthBuffer(pixelCount, 1.0f);
	std::vector<char> printBuffer(sizeof(Pixel) * pixels.size() + 1);

	char ch;
	do
    {
		ch = getch_noblock();
        a = std::chrono::system_clock::now();
        std::chrono::duration<double, std::milli> work_time = a - b;

        if (work_time.count() < frameTime)
        {
            std::chrono::duration<double, std::milli> delta_ms(frameTime - work_time.count());
            auto delta_ms_duration = std::chrono::duration_cast<std::chrono::milliseconds>(delta_ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(delta_ms_duration.count()));
        }

        b = std::chrono::system_clock::now();
        std::chrono::duration<double, std::milli> sleep_time = b - a;

		// run frame
		MoveCursorToStart();
		std::fill(depthBuffer.begin(), depthBuffer.end(), 1.0f);
		for (std::size_t i = 0; i < pixelCount; i++) {
			char color[19] = {'\x1b', '[', '3', '8', ';', '2', ';', '2', '5', '5', ';', '2', '5', '5', ';', '2', '5', '5', 'm'};
			memcpy(pixels[i].colorCode, color, sizeof(color));
			pixels[i].pixel = noPixelChar;
		}

		for (std::size_t i = 0; i < 4; i++) {
			Vertex& vertex = vertices[i];
			vertex.pos = rotate(vertex.pos, Vec3{0.0f, 0.0f, -1.0f}, 0.1f);
		}

		// rasterize triangles
		for (std::size_t i = 0; i < indices.size(); i+=3) {
			rasterize_triangle(texture, pixels, depthBuffer, vertices[indices[i]], vertices[indices[i+1]], vertices[indices[i+2]], terminal);
		}

		// print
		memcpy(printBuffer.data(), pixels.data(), sizeof(Pixel) * pixels.size());
		printBuffer[sizeof(Pixel) * pixels.size()] = '\0';
		std::cout << printBuffer.data();
    } while (ch != 'q');

#ifdef __linux__
	// restore the previous terminal
	std::cout << RESTORE_TERMINAL;	
#endif 
	return 0;
}
