#include "OutputFormat.h"

#include <stdexcept>

namespace ffpp
{

OutputFormat::OutputFormat(const AVOutputFormat *avp): avp(avp)
{}

OutputFormat::OutputFormat(const char* name, const char* filename): avp(guess_format(name, filename, NULL))
{
	if (!avp) throw(std::runtime_error("Could not guess the output format"));
}

const char* OutputFormat::GetName() const
{
	if (!avp) return 0;
	return avp->name;
}

const AVOutputFormat* OutputFormat::get() const
{
	return avp;
}

}
