#pragma once
#include <queue>
#include <mutex>
#include <vector>

//-----------------------------------------------------------------------------------------------
class	JobSystem;
extern	JobSystem* g_theJobSystem;

//-----------------------------------------------------------------------------------------------
enum class JobStatus
{
	QUEUED,
	CLAIMED,
	COMPLETED,
	RETREIVED,
};

//-----------------------------------------------------------------------------------------------
class Job
{
public:
	virtual ~Job() = default;
	virtual void Execute() = 0;

public:
	std::atomic<JobStatus> m_jobStatus = JobStatus::QUEUED;
};

//-----------------------------------------------------------------------------------------------
class JobWorker
{
public:
	JobWorker(JobSystem* jobSystem, int workerThreadID);
	~JobWorker();

	static void ThreadMain(int workerID);

private:
	JobSystem*		m_jobSystem = nullptr;
	std::thread*	m_thread = nullptr;
	int				m_workerThreadID = -1;
};

//-----------------------------------------------------------------------------------------------
struct JobSystemConfig
{
	int m_preferredNumberOfWorkers = -1;
};

//-----------------------------------------------------------------------------------------------
class JobSystem
{
public:
	JobSystem(JobSystemConfig const& config);
	~JobSystem();
	void Startup();
	void BeginFrame();
	void EndFrame();
	void Shutdown();
	bool IsQuitting();

	void CreateWorkers(int numWorkerThreads);
	void PostNewJob(Job* job);
	void MoveJobToCompletedList(Job* job);
	Job* ClaimJobToExecute();
	Job* RetreiveCompletedJob();


private:
	std::vector<JobWorker*> m_workers;

	std::deque<Job*>		m_unclaimedJobsList;
	std::mutex				m_unclaimedJobsListMutex;

	std::deque<Job*>		m_claimedJobsList;
	std::mutex				m_claimedJobsListMutex;

	std::deque<Job*>		m_completedJobsList;
	std::mutex				m_completedJobsListMutex;

	JobSystemConfig			m_config;
	std::atomic<bool>		m_isQuitting = false;
};
