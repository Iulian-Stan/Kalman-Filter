#ifndef _csvRow_
#define _csvRow_

#include <vector>

class CSVRow
{
public:
	double const& operator[](std::size_t index) const;
	std::size_t size() const;
	void readNextRow(std::istream& str);
private:
	std::vector<double> m_data;
};

#endif // _csvRow_
