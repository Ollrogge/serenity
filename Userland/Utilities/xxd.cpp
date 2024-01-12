#include <AK/AllOf.h>
#include <AK/CharacterTypes.h>
#include <LibCore/ArgsParser.h>
#include <LibCore/File.h>
#include <LibCore/System.h>
#include <LibMain/Main.h>
#include <AK/String.h>

static constexpr size_t COLS_AMT = 16;
static constexpr size_t COLS_AMT_C = 12;

enum class State {
    Default,
    CStyle,
};

static void
print_line(Bytes line, size_t offset, size_t cols_amt_setting)
{
    out("{:08}: ", offset);
    for (size_t i = 0; i < cols_amt_setting; i += 2) {
        if (i + 1 < line.size()) {
            auto chunk = line.slice(i, 2);
            out("{:02x}{:02x} ", chunk[0], chunk[1]);
        } else if (i < line.size()) {
            out("{:02x}   ", line[i]);
        } else {
            out("     ");
        }
    }

    out(" ");

    for (auto const& byte : line) {
        if (is_ascii_printable(byte)) {
            putchar(byte);
        } else {
            putchar('.');
        }
    }

    putchar('\n');
}

static void print_line_c_style(Bytes line)
{
    out("  ");
    for (size_t i = 0; i < line.size(); ++i) {
            out("0x{:02x}, ", line[i]);
    }

    putchar('\n');
}

static ErrorOr<String> path_to_variable_name(StringView path)
{
    auto work = path.to_byte_string();

    work = work.replace("."sv, "_"sv, ReplaceMode::All);
    work = work.replace("/"sv, "_"sv, ReplaceMode::All);

    return String::from_byte_string(work);
}

ErrorOr<int> serenity_main(Main::Arguments args)
{
    Core::ArgsParser args_parser;
    StringView path;
    bool autoskip = false;
    bool c_include_file_style = false;

    args_parser.add_positional_argument(path, "Input file", "input", Core::ArgsParser::Required::No);
    args_parser.add_option(autoskip, "A single '*' replaces NUL-lines.", "autoskip", 'a');
    args_parser.add_option(c_include_file_style, "Output in C include file style.", "include", 'i');
    args_parser.parse(args);

    auto file = TRY(Core::File::open_file_or_standard_stream(path, Core::File::OpenMode::Read));

    Array<u8, BUFSIZ> contents;
    Bytes bytes;
    size_t total_bytes_read = 0x0;
    auto state = State::Default;
    size_t cols_amt_setting = COLS_AMT;

    if (c_include_file_style) {
        cols_amt_setting = COLS_AMT_C;
        state = State::CStyle;
        auto variable_name = TRY(path_to_variable_name(path));
        outln("u8 {}[] = {{", variable_name);
    }

    const size_t max_read_size = contents.size() - contents.size() % cols_amt_setting;

    bool is_input_remaining = true;
    while (is_input_remaining) {
        auto bytes_to_read = max_read_size - bytes.size();

        bytes = contents.span().slice(0, bytes_to_read);
        bytes = TRY(file->read_some(bytes));

        total_bytes_read += bytes.size();

        if (bytes.size() < bytes_to_read) {
            is_input_remaining = false;
        }

        while (bytes.size() > 0) {
            auto line_len = bytes.size() > cols_amt_setting ? cols_amt_setting : bytes.size();
            auto current_line = bytes.slice(0, line_len);
            bytes = bytes.slice(line_len);

            if (autoskip && all_of(bytes, [](auto& b) { return b == 0x0; })) {
                outln("*");
                continue;
            }

            switch (state) {
            case State::Default:
                print_line(current_line, total_bytes_read, cols_amt_setting);
                break;
            case State::CStyle:
                print_line_c_style(current_line);
                break;
            }
        }
    }

    if (state == State::CStyle) {
        outln("}};");
        auto variable_name = TRY(path_to_variable_name(path));
        outln("size_t {}_len = {};", variable_name, total_bytes_read);
    }

    return 0;
}
