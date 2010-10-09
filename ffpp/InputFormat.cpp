#include "InputFormat.h"

namespace ffpp
{

InputFormat::InputFormat(const AVInputFormat *avp): avp(avp)
{}

const char* InputFormat::GetName() const
{
	if (!avp) return 0;
	return avp->name;
}

const AVInputFormat* InputFormat::get() const
{
	return avp;
}

}
