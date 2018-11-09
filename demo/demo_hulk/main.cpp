#include <thread>
#include "aris.h"

int main()
{
	aris::control::EthercatMaster mst;
	
	// 自动扫描从站 //
	mst.scan();

	// 设置实时任务 //
	mst.setControlStrategy([&]()
	{
		// 每过一秒，切换一下IO输出 //
		static int count{ 0 };
		static std::uint32_t value{ 0xffffffff };
		if (++count % 1000 == 0)
		{
			// 写PDO //
			mst.ecSlavePool()[0].writePdo(0x7000, 0x01, value);

			// 切换IO状态 //
			value = ~value;
		}
	});

	// 开启实时任务 //
	mst.start();

	// 这里执行100秒 //
	std::this_thread::sleep_for(std::chrono::seconds(100));

	// 结束实时任务 //
	mst.stop();

	return 0;
}

