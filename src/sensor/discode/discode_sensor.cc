/*
 * discode_sensor.cc
 *
 *  Created on: Oct 30, 2010
 *      Author: mboryn
 */

#include <stdexcept>
#include <cstdio>

#include <sys/uio.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include "discode_sensor.h"
#include "base/lib/logger.h"
#include "base/lib/configurator.h"
#include "headers.h"

namespace mrrocpp {

namespace ecp_mp {

namespace sensor {

namespace discode {

using namespace std;
using namespace logger;
using namespace boost;
using lib::configurator;

discode_sensor::discode_sensor(mrrocpp::lib::configurator& config, const std::string& section_name) :
	config(config), section_name(section_name)
{
	base_period = current_period = 1;
	timer.print_last_status();
	fflush(stdout);

	header_iarchive = shared_ptr <xdr_iarchive <> > (new xdr_iarchive <> );
	iarchive = shared_ptr <xdr_iarchive <> > (new xdr_iarchive <> );
	header_oarchive = shared_ptr <xdr_oarchive <> > (new xdr_oarchive <> );
	oarchive = shared_ptr <xdr_oarchive <> > (new xdr_oarchive <> );

	reading_message_header rmh;

	*header_oarchive << rmh;
	reading_message_header_size = header_oarchive->getArchiveSize();
	header_oarchive->clear_buffer();

	timer_init();
}

discode_sensor::~discode_sensor()
{

}

void discode_sensor::configure_sensor()
{
	timer_show("discode_sensor::configure_sensor() begin");

	// Retrieve FraDIA node name and port from configuration file.
	uint16_t discode_port = config.value <uint16_t> ("discode_port", section_name);
	const std::string discode_node_name = config.value <std::string> ("discode_node_name", section_name);

	// Try to open socket.
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd == -1) {
		throw std::runtime_error("socket(): " + std::string(strerror(errno)));
	}

	int flag = 1;
	if (setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int)) == -1) {
		throw std::runtime_error("setsockopt(): " + std::string(strerror(errno)));
	}

	// Get server hostname.
	hostent * server = gethostbyname(discode_node_name.c_str());
	if (server == NULL) {
		throw std::runtime_error("gethostbyname(" + discode_node_name + "): " + std::string(hstrerror(h_errno)));
	}

	// Data with address of connection
	sockaddr_in serv_addr;

	// Reset socketaddr data.
	memset(&serv_addr, 0, sizeof(serv_addr));

	// Fill it with data.
	serv_addr.sin_family = AF_INET;
	memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);
	serv_addr.sin_port = htons(discode_port);

	// Try to establish a connection with FraDIA.
	if (connect(sockfd, (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) == -1) {
		throw std::runtime_error("connect(): " + std::string(strerror(errno)));
	}

	timer_show("discode_sensor::configure_sensor() end");
}

void discode_sensor::initiate_reading()
{
	timer_show("discode_sensor::initiate_reading() begin");

	log_dbg("oarchive.getArchiveSize(): %d\n", oarchive->getArchiveSize());

	initiate_message_header imh;
	imh.data_size = oarchive->getArchiveSize();

	header_oarchive->clear_buffer();
	*header_oarchive << imh;

	struct iovec iov[2];
	ssize_t nwritten;

	iov[0].iov_base = (void*) header_oarchive->get_buffer();
	iov[0].iov_len = header_oarchive->getArchiveSize();
	iov[1].iov_base = (void*) oarchive->get_buffer();
	iov[1].iov_len = oarchive->getArchiveSize();

	nwritten = writev(sockfd, iov, 2);
	if (nwritten == -1) {
		throw runtime_error("writev(sockfd, iov, 2) == -1");
	}
	if (nwritten != header_oarchive->getArchiveSize() + oarchive->getArchiveSize()) {
		throw runtime_error("writev(sockfd, iov, 2) != header_oarchive->getArchiveSize() + oarchive->getArchiveSize()");
	}

	oarchive->clear_buffer();

	timer_show("discode_sensor::initiate_reading() end");
}

void discode_sensor::get_reading()
{
	timer_show("discode_sensor::get_reading() begin");

	header_iarchive->clear_buffer();
	int nread = read(sockfd, header_iarchive->get_buffer(), reading_message_header_size);
	if (nread == -1) {
		throw runtime_error(string("read() failed: ") + strerror(errno));
	}
	if (nread != reading_message_header_size) {
		throw runtime_error("read() failed: nread != reading_message_header_size");
	}

	reading_message_header rmh;
	*header_iarchive >> rmh;

	iarchive->clear_buffer();
	nread = read(sockfd, iarchive->get_buffer(), rmh.data_size);
	if (nread == -1) {
		throw runtime_error(string("read() failed: ") + strerror(errno));
	}
	if (nread != rmh.data_size) {
		throw runtime_error("read() failed: nread != rmh.data_size");
	}

	timer_show("discode_sensor::get_reading() end");
}

void discode_sensor::terminate()
{
	close(sockfd);
}

shared_ptr <xdr_iarchive <> > discode_sensor::get_iarchive()
{
	return iarchive;
}

shared_ptr <xdr_oarchive <> > discode_sensor::get_oarchive()
{
	return oarchive;
}

void discode_sensor::timer_init()
{
	if (timer.start() != mrrocpp::lib::timer::TIMER_STARTED) {
		timer.print_last_status();
		fflush(stdout);
		throw logic_error("discode_sensor::configure_sensor(): timer.start() != mrrocpp::lib::timer::TIMER_STARTED");
	}
}

void discode_sensor::timer_show(const char *str)
{
	if (timer.stop() != mrrocpp::lib::timer::TIMER_STOPPED) {
		timer.print_last_status();
		fflush(stdout);
		throw logic_error("discode_sensor::get_reading(): timer.stop() != mrrocpp::lib::timer::TIMER_STOPPED");
	}
	float seconds;
	if (timer.get_time(seconds) != mrrocpp::lib::timer::TIME_RETRIVED) {
		timer.print_last_status();
		fflush(stdout);
		throw logic_error("discode_sensor::get_reading(): timer.get_time(seconds) != mrrocpp::lib::timer::TIME_RETRIVED");
	}
	log_dbg("Time elapsed (in %s): %g s\n", str, seconds);

	if (timer.start() != mrrocpp::lib::timer::TIMER_STARTED) {
		timer.print_last_status();
		fflush(stdout);
		throw logic_error("discode_sensor::get_reading(): timer.start() != mrrocpp::lib::timer::TIMER_STARTED");
	}
}

} // namespace discode

} // namespace mrrocpp

} // namespace ecp_mp

} // namespace mrrocpp
